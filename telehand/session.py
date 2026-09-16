"""One-instance guard, so a run that holds the hand can always be found again.

The hand and the camera are single-access devices, and a teleop loop that is
still engaged keeps driving motors whether or not anyone is watching. Recording
the pid of the process that actually owns them means `run.py stop` can end that
run cleanly instead of leaving it orphaned -- signalling a wrapper shell does
not reach the Python process underneath it.
"""

from __future__ import annotations

import contextlib
import json
import os
import signal
import time
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# This is the instrumented copy of the project. The hand is still a single
# device, so the lock has to be shared with the original checkout -- otherwise
# each copy guards only itself and both could drive the motors at once. Fall
# back to a local lock if the original is not there.
_ORIGINAL = Path("/home/xu-square/telehand")
LOCK_PATH = ((_ORIGINAL if _ORIGINAL.is_dir() else PROJECT_ROOT) / ".telehand.lock")


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def read_lock(path: Path = LOCK_PATH) -> Optional[dict]:
    """Return the running session's record, clearing the file if it is stale."""
    path = Path(path)
    if not path.exists():
        return None
    try:
        info = json.loads(path.read_text())
        pid = int(info["pid"])
    except (ValueError, KeyError, OSError):
        path.unlink(missing_ok=True)
        return None
    if not _alive(pid):
        path.unlink(missing_ok=True)
        return None
    return info


class SessionBusy(RuntimeError):
    def __init__(self, info: dict):
        self.info = info
        started = time.strftime("%H:%M:%S", time.localtime(info.get("started", 0)))
        super().__init__(
            f"another telehand session is already running: "
            f"`{info.get('command', '?')}` (pid {info['pid']}, started {started}).\n"
            f"  Stop it with:  python3 run.py stop\n"
            f"  It holds the serial port and camera, so this run cannot start."
        )


@contextlib.contextmanager
def hold(command: str, **details):
    """Claim the hardware for this process, releasing on any exit path."""
    existing = read_lock()
    if existing is not None:
        raise SessionBusy(existing)

    LOCK_PATH.write_text(json.dumps({
        "pid": os.getpid(),
        "command": command,
        "started": time.time(),
        **details,
    }, indent=2) + "\n")

    # `stop` sends SIGINT, but a plain SIGTERM should shut down just as cleanly
    # rather than killing the process with motors still enabled.
    previous = signal.getsignal(signal.SIGTERM)

    def _term(signum, frame):
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, _term)
    except ValueError:
        previous = None  # not on the main thread; leave the handler alone

    try:
        yield
    finally:
        if previous is not None:
            with contextlib.suppress(ValueError):
                signal.signal(signal.SIGTERM, previous)
        with contextlib.suppress(OSError):
            info = json.loads(LOCK_PATH.read_text())
            if int(info.get("pid", -1)) == os.getpid():
                LOCK_PATH.unlink(missing_ok=True)


def stop(timeout: float = 15.0) -> int:
    """Interrupt the running session and wait for it to release the hardware."""
    info = read_lock()
    if info is None:
        print("No telehand session is running.")
        return 0

    pid = int(info["pid"])
    print(f"Stopping `{info.get('command', '?')}` (pid {pid}) ...")

    # SIGINT so the command's own KeyboardInterrupt path runs: motors stopped,
    # monitor thread joined, serial closed.
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        LOCK_PATH.unlink(missing_ok=True)
        print("Already gone.")
        return 0

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not _alive(pid):
            LOCK_PATH.unlink(missing_ok=True)
            print("Stopped cleanly; serial port and camera released.")
            return 0
        time.sleep(0.25)

    print(f"Still running after {timeout:.0f}s; sending SIGTERM.")
    with contextlib.suppress(ProcessLookupError):
        os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not _alive(pid):
            LOCK_PATH.unlink(missing_ok=True)
            print("Stopped.")
            return 0
        time.sleep(0.25)

    print(f"pid {pid} did not exit. It may still hold the hand.")
    print(f"  Force it with:  kill -9 {pid}")
    print("  Note that a forced kill leaves the motors enabled.")
    return 1
