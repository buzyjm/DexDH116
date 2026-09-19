"""One module per CLI subcommand; each exposes ``run(args) -> int``.

``telehand.cli`` imports a command module only when that command is invoked,
so a command such as ``stop`` never pays for importing OpenCV or MediaPipe.
"""
