#!/usr/bin/env python3
"""Fetch the exact approved CC0 building texture bytes; stdlib only.

Source files remain outside Git (D14). The manifest pins URLs, lengths and SHA256;
no 'latest' API request occurs during installation. --verify is network-free.
Selected original facade photographs come from one pinned CC0 source archive;
only approved members are installed and the source archive is not shipped.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.request
import zipfile


def verified(path, entry):
    return path.is_file() and path.stat().st_size == entry["size"] and hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]


def checked_bytes(data, entry):
    if len(data) != entry["size"] or hashlib.sha256(data).hexdigest() != entry["sha256"]:
        raise ValueError("asset length/hash mismatch: " + entry["path"])
    return data


def install(manifest, destination, verify_only=False):
    destination = destination.resolve()
    missing = []
    pending = []
    for entry in manifest["files"]:
        path = (destination / entry["path"]).resolve()
        if not path.is_relative_to(destination):
            raise ValueError("asset path escapes destination")
        if not 0 < entry["size"] <= 16 * 1024 * 1024:
            raise ValueError("invalid bounded texture size")
        if verified(path, entry):
            continue
        if verify_only:
            missing.append(entry["path"])
        else:
            pending.append((entry, path))
    if missing:
        raise ValueError("missing or changed assets: " + ", ".join(missing))
    archives = {entry["id"]: entry for entry in manifest.get("archives", [])}
    with tempfile.TemporaryDirectory(prefix="rg-facade-sources-") as scratch:
        opened = {}
        try:
            for entry, path in pending:
                if "archive" in entry:
                    identity = entry["archive"]
                    archive = archives.get(identity)
                    allowed = "https://opengameart.org/sites/default/files/buildings-apartments-shopfronts-shutters.zip"
                    if not archive or archive["url"] != allowed:
                        raise ValueError("unapproved archive origin")
                    if not 0 < archive["size"] <= 240 * 1024 * 1024:
                        raise ValueError("invalid bounded archive size")
                    member = entry["member"]
                    if not member.startswith("buildings-apartments-shopfronts-shutters/") or ".." in Path(member).parts:
                        raise ValueError("unapproved archive member")
                    if identity not in opened:
                        local = Path(scratch) / "urban_facades.zip"
                        digest = hashlib.sha256()
                        count = 0
                        req = urllib.request.Request(archive["url"], headers={"User-Agent": "racing_game-terrain-assets/1"})
                        with urllib.request.urlopen(req, timeout=60) as response, local.open("wb") as output:
                            if response.geturl() != allowed:
                                raise ValueError("unapproved archive redirect")
                            while block := response.read(1024 * 1024):
                                count += len(block)
                                if count > archive["size"]:
                                    raise ValueError("archive length mismatch")
                                digest.update(block)
                                output.write(block)
                        if count != archive["size"] or digest.hexdigest() != archive["sha256"]:
                            raise ValueError("archive length/hash mismatch")
                        opened[identity] = zipfile.ZipFile(local)
                    source = opened[identity]
                    if source.getinfo(member).file_size != entry["size"]:
                        raise ValueError("archive member length mismatch")
                    data = checked_bytes(source.read(member), entry)
                else:
                    if not entry["url"].startswith("https://dl.polyhaven.org/file/ph-assets/"):
                        raise ValueError("unapproved texture origin")
                    req = urllib.request.Request(entry["url"], headers={"User-Agent": "racing_game-terrain-assets/1"})
                    with urllib.request.urlopen(req, timeout=45) as response:
                        if not response.geturl().startswith("https://dl.polyhaven.org/"):
                            raise ValueError("unapproved redirect")
                        data = checked_bytes(response.read(entry["size"] + 1), entry)
                path.parent.mkdir(parents=True, exist_ok=True)
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
        finally:
            for source in opened.values():
                source.close()


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
