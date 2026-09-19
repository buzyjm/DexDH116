#!/usr/bin/env python3
"""Command-line entry point.

Puts the project-local ``vendor/`` directory on ``sys.path`` and hands over to
``telehand.cli``. Run ``python3 run.py --help`` for the list of commands.
"""

import sys

from telehand.bootstrap import bootstrap

bootstrap()

from telehand.cli import main  # noqa: E402  (needs vendor/ on the path first)

if __name__ == "__main__":
    sys.exit(main())
