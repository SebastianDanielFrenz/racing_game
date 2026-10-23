#!/usr/bin/env python3
"""Fetch the exact approved CC0 building texture bytes; stdlib only.

Source files remain outside Git (D14). The manifest pins URLs, lengths and SHA256;
no 'latest' API request occurs during installation. --verify is network-free.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.request


def verified(path, entry):
    return path.is_file() and path.stat().st_size == entry["size"] and hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]


def install(manifest, destination, verify_only=False):
    destination = destination.resolve()
    missing = []
    for entry in manifest["files"]:
        path = (destination / entry["path"]).resolve()
        if not path.is_relative_to(destination):
            raise ValueError("asset path escapes destination")
        if verified(path, entry):
            continue
        if verify_only:
            missing.append(entry["path"])
            continue
        if not entry["url"].startswith("https://dl.polyhaven.org/file/ph-assets/"):
            raise ValueError("unapproved texture origin")
        if not 0 < entry["size"] <= 16 * 1024 * 1024:
            raise ValueError("invalid bounded texture size")
        path.parent.mkdir(parents=True, exist_ok=True)
        req = urllib.request.Request(entry["url"], headers={"User-Agent": "racing_game-terrain-assets/1"})
        with urllib.request.urlopen(req, timeout=45) as response:
            if not response.geturl().startswith("https://dl.polyhaven.org/"):
                raise ValueError("unapproved redirect")
            data = response.read(entry["size"] + 1)
        if len(data) != entry["size"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise ValueError("asset length/hash mismatch: " + entry["path"])
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".part", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(data)
            temporary.replace(path)
        finally:
            if temporary and temporary.exists():
                temporary.unlink()
        print("installed", entry["path"])
    if missing:
        raise ValueError("missing or changed assets: " + ", ".join(missing))


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="verify installed bytes without network access")
    parser.add_argument("--destination", type=Path, default=root / "data/textures/buildings")
    args = parser.parse_args()
    manifest = json.loads((root / "data/textures/buildings/assets.json").read_text(encoding="utf-8"))
    if manifest.get("format") != "rg.building_assets/1":
        raise ValueError("unsupported building asset manifest")
    install(manifest, args.destination, args.verify)
    print("building assets verified:", len(manifest["files"]))


if __name__ == "__main__":
    main()
