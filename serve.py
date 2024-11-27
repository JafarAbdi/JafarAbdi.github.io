#!/usr/bin/env -S uv run --script
# /// script
# dependencies = ["markdown", "pygments", "watchfiles"]
# ///
"""Serve _site locally and rebuild whenever a source file changes."""

import functools
import http.server
import pathlib
import sys
import threading
import traceback

import watchfiles

import build


def rebuild() -> None:
    try:
        build.main()
    except Exception:  # noqa: BLE001 - keep serving whatever the build raises
        traceback.print_exc()


def is_source(change: watchfiles.Change, path: str) -> bool:
    parts = pathlib.Path(path).parts
    return "_site" not in parts and ".git" not in parts


def main() -> None:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    rebuild()
    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler, directory=str(build.OUT)
    )
    server = http.server.ThreadingHTTPServer(("", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"serving http://localhost:{port} (ctrl-c to stop)")
    for changes in watchfiles.watch(
        build.ROOT, watch_filter=is_source, raise_interrupt=False
    ):
        for _, path in sorted(changes):
            print(f"changed: {pathlib.Path(path).relative_to(build.ROOT)}")
        rebuild()


if __name__ == "__main__":
    main()
