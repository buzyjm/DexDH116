"""Experimental components. Not part of the stable teleop path.

Nothing here is imported by the production pipeline
(tracking -> scalar FK retargeter -> safety/filtering -> hand) unless an
explicitly experimental flag or subcommand asks for it:

* ``framemap``      frame-based joint mapping (``teleop --pose-mode frames``),
                    tried and found less stable than the FK solver; kept for
                    future comparison work.
* ``frameviz``      replay visualiser and stability metrics for the wrist-local
                    hand frames (``run.py frames``).
* ``stepresponse``  DH116 actuator characterisation (``run.py stepresponse``);
                    this one moves the hand.
* ``cli``           the subcommands and options that expose the above.

Findings from these tools live in docs/EXPERIMENTS.md.
"""
