"""``cameras``: list capture devices and remember one as the default."""

from __future__ import annotations

import time

from ..camera import Camera, list_cameras, resolve_camera, save_camera, saved_camera


def run(args) -> int:
    """List capture devices, with a measured frame rate for each."""
    cams = list_cameras()
    if not cams:
        print("No capture devices found.")
        return 1
    if args.use is not None:
        node = resolve_camera(args.use)
        chosen = next((c for c in cams if c["node"] == str(node)
                       or c["index"] == node), None)
        # Prefer the by-path link: device numbers move, USB ports do not.
        spec = (chosen or {}).get("by_path") or str(node)
        save_camera(spec)
        print(f"default camera set to {spec}")
        if chosen:
            print(f"  ({chosen['name']}, currently {chosen['node']})")
        return 0
    current = saved_camera()
    if current:
        print(f"saved default: {current}\n")
    print(f"{'index':>5}  {'node':<13}{'name':<34}measured")
    for c in cams:
        fps = brightness = float("nan")
        try:
            cam = Camera(c["node"], settle=args.settle)
            t0 = time.monotonic()
            n, frame = 0, None
            while n < 45 and time.monotonic() - t0 < 4.0:
                ok, f = cam.read()
                if ok:
                    n += 1
                    frame = f
            fps = n / max(time.monotonic() - t0, 1e-6)
            brightness = float(frame.mean()) if frame is not None else float("nan")
            cam.release()
        except Exception as exc:
            print(f"{c['index']:>5}  {c['node']:<13}{c['name']:<34}unavailable: {exc}")
            continue
        print(f"{c['index']:>5}  {c['node']:<13}{c['name']:<34}{fps:5.1f} fps  brightness {brightness:5.1f}")
        if c["by_path"]:
            print(f"       stable: {c['by_path']}")
    print("\nDevice numbers move when cameras are replugged. Pass a stable path or a "
          "name fragment:\n  python3 run.py teleop --camera /dev/v4l/by-path/...-index0"
          "\n  python3 run.py teleop --camera icspring")
    return 0
