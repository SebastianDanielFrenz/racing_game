"""Network-free checks of the pinned building asset installer."""
import hashlib
import io
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("building_assets", ROOT / "tools/fetch_building_assets.py")
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


class BuildingAssetsTest(unittest.TestCase):
    def entry(self, data=b"texture"):
        return {"path": "wall/test.jpg", "size": len(data), "sha256": hashlib.sha256(data).hexdigest(),
                "url": "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/1k/test.jpg"}

    def test_installed_bytes_verify_without_network(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            entry = self.entry()
            path = root / entry["path"]
            path.parent.mkdir()
            path.write_bytes(b"texture")
            with patch.object(assets.urllib.request, "urlopen", side_effect=AssertionError("network forbidden")):
                assets.install({"files": [entry]}, root, True)
                path.write_bytes(b"changed")
                with self.assertRaisesRegex(ValueError, "missing or changed"):
                    assets.install({"files": [entry]}, root, True)

    def test_manifest_cannot_write_outside_asset_root(self):
        with tempfile.TemporaryDirectory() as folder:
            entry = self.entry()
            entry["path"] = "../outside.jpg"
            with self.assertRaisesRegex(ValueError, "escapes"):
                assets.install({"files": [entry]}, Path(folder))

    def test_unapproved_origin_is_rejected_before_network(self):
        with tempfile.TemporaryDirectory() as folder:
            entry = self.entry()
            entry["url"] = "http://other.example/texture"
            with patch.object(assets.urllib.request, "urlopen", side_effect=AssertionError("network forbidden")):
                with self.assertRaisesRegex(ValueError, "unapproved"):
                    assets.install({"files": [entry]}, Path(folder))

    def archive_manifest(self, member_data=b"selected texture"):
        member = "buildings-apartments-shopfronts-shutters/facade.jpg"
        archive_bytes = io.BytesIO()
        with zipfile.ZipFile(archive_bytes, "w", compression=zipfile.ZIP_STORED) as source:
            source.writestr(member, member_data)
            source.writestr("unlisted/ignored.jpg", b"unlisted texture")
        payload = archive_bytes.getvalue()
        archive_url = "https://opengameart.org/sites/default/files/buildings-apartments-shopfronts-shutters.zip"
        archive = {
            "id": "facades",
            "url": archive_url,
            "size": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
        entry = {
            "path": "facades/installed.jpg",
            "size": len(member_data),
            "sha256": hashlib.sha256(member_data).hexdigest(),
            "archive": "facades",
            "member": member,
        }
        return {"files": [entry], "archives": [archive]}, payload, archive_url

    def response_for(self, payload, url):
        class Response(io.BytesIO):
            def geturl(self):
                return url
        return Response(payload)

    def test_archive_installs_only_selected_pinned_member(self):
        manifest, payload, url = self.archive_manifest()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with patch.object(assets.urllib.request, "urlopen", return_value=self.response_for(payload, url)):
                assets.install(manifest, root)
            self.assertEqual((root / "facades/installed.jpg").read_bytes(), b"selected texture")
            self.assertFalse((root / "unlisted/ignored.jpg").exists())

    def test_corrupt_archive_hash_preserves_previous_file(self):
        manifest, payload, url = self.archive_manifest()
        manifest["archives"][0]["sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            destination = root / manifest["files"][0]["path"]
            destination.parent.mkdir(parents=True)
            destination.write_bytes(b"previous bytes")
            with patch.object(assets.urllib.request, "urlopen", return_value=self.response_for(payload, url)):
                with self.assertRaisesRegex(ValueError, "archive length/hash mismatch"):
                    assets.install(manifest, root)
            self.assertEqual(destination.read_bytes(), b"previous bytes")
            self.assertFalse(list(destination.parent.glob("*.part")))

    def test_archive_member_length_or_hash_mismatch_rejects(self):
        for mismatch in ("length", "hash"):
            with self.subTest(mismatch=mismatch):
                manifest, payload, url = self.archive_manifest()
                if mismatch == "length":
                    manifest["files"][0]["size"] += 1
                    error = "archive member length mismatch"
                else:
                    manifest["files"][0]["sha256"] = "0" * 64
                    error = "asset length/hash mismatch"
                with tempfile.TemporaryDirectory() as folder:
                    root = Path(folder)
                    destination = root / manifest["files"][0]["path"]
                    destination.parent.mkdir(parents=True)
                    destination.write_bytes(b"previous bytes")
                    with patch.object(assets.urllib.request, "urlopen", return_value=self.response_for(payload, url)):
                        with self.assertRaisesRegex(ValueError, error):
                            assets.install(manifest, root)
                    self.assertEqual(destination.read_bytes(), b"previous bytes")
                    self.assertFalse(list(destination.parent.glob("*.part")))

    def test_corrupt_download_does_not_replace_previous_file(self):
        class Response:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def geturl(self): return "https://dl.polyhaven.org/file/ph-assets/texture.jpg"
            def read(self, limit): return b"corrupt"
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            entry = self.entry()
            path = root / entry["path"]
            path.parent.mkdir()
            path.write_bytes(b"previous bytes")
            with patch.object(assets.urllib.request, "urlopen", return_value=Response()):
                with self.assertRaisesRegex(ValueError, "mismatch"):
                    assets.install({"files": [entry]}, root)
            self.assertEqual(path.read_bytes(), b"previous bytes")
            self.assertFalse(list(path.parent.glob("*.part")))


if __name__ == "__main__":
    unittest.main()
