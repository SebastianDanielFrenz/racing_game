"""Network-free checks of the pinned building asset installer."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

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
