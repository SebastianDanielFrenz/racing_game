"""Focused tests for the self-contained installer payloads; no Steam/network writes."""
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SOURCE = pathlib.Path(__file__).parents[1] / 'install_linux.sh'


def payload(start, end):
    text = SOURCE.read_text(encoding='utf-8-sig')
    return text.split(start + '\n', 1)[1].split('\n' + end + '\n', 1)[0]


STEAM = payload('cat > "$STATE_DIR/register_steam.py" <<\'STEAM\'', 'STEAM')
ART = payload('python3 - "$STATE_DIR" <<\'ART\'', 'ART')


class SteamRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home = pathlib.Path(self.tmp.name)
        self.steam = self.home / '.local/share/Steam'
        self.config = self.steam / 'userdata/42/config'
        self.config.mkdir(parents=True)
        self.state = self.home / 'racing game/.local-install'
        (self.state / 'artwork').mkdir(parents=True)
        (self.state / 'config.json').write_text('{"mode":"vr"}')
        for name in ['header', 'capsule', 'hero', 'icon']:
            (self.state / f'artwork/{name}.png').write_bytes(b'artwork-' + name.encode())
        self.ns = {'__name__': 'installer_test'}
        exec(compile(STEAM, 'register_steam.py', 'exec'), self.ns)
        self.ns['steam_running'] = lambda: False
        self.home_patch = patch.object(pathlib.Path, 'home', return_value=self.home)
        self.home_patch.start()
        self.addCleanup(self.home_patch.stop)

    def register(self):
        self.ns['register'](self.state)
        return self.ns['read_vdf']((self.config / 'shortcuts.vdf').read_bytes())

    def test_add_update_preserve_other_game_and_user_settings(self):
        other = {'appid': (2, struct.pack('<I', 123)), 'AppName': (1, 'Other Game'),
                 'Exe': (1, 'other'), 'unknown_float': (3, b'\x00\x00\x80\x3f'),
                 'unknown_u64': (7, b'12345678')}
        original = {'shortcuts': (0, {'0': (0, other)})}
        (self.config / 'shortcuts.vdf').write_bytes(self.ns['write_vdf'](original))
        tree = self.register()
        self.assertEqual(other, tree['shortcuts'][1]['0'][1])
        self.assertEqual(len(tree['shortcuts'][1]), 2)
        entry = tree['shortcuts'][1]['1'][1]
        appid = struct.unpack('<I', entry['appid'][1])[0]
        self.assertEqual(entry['Exe'], (1, '"' + str(self.state / 'launch.py') + '"'))
        self.assertTrue((self.config / f'grid/{appid}p.png').exists())
        self.assertTrue((self.config / f'grid/{appid}_hero.png').exists())
        self.assertTrue((self.home / '.local/share/applications/racing-game.desktop').exists())
        entry['LaunchOptions'] = (1, '--desktop --flat')
        entry['AppName'] = (1, 'My Racing Game')
        entry['tags'] = (0, {'0': (1, 'My category')})
        (self.config / 'shortcuts.vdf').write_bytes(self.ns['write_vdf'](tree))
        updated = self.register()
        self.assertEqual(updated, tree)
        self.assertTrue(list(self.config.glob('shortcuts.vdf.racing-game-*.bak')))

    def test_live_steam_leaves_files_untouched(self):
        self.ns['steam_running'] = lambda: True
        with self.assertRaisesRegex(SystemExit, 'Fully exit Steam'):
            self.register()
        self.assertFalse((self.config / 'shortcuts.vdf').exists())

    def test_malformed_file_is_not_rewritten(self):
        path = self.config / 'shortcuts.vdf'
        path.write_bytes(b'\x09shortcuts\0garbage')
        with self.assertRaises(ValueError):
            self.register()
        self.assertEqual(path.read_bytes(), b'\x09shortcuts\0garbage')

    def test_ambiguous_accounts_refused(self):
        (self.steam / 'userdata/73/config').mkdir(parents=True)
        with self.assertRaisesRegex(SystemExit, 'Cannot select Steam account'):
            self.register()
        self.assertFalse((self.config / 'shortcuts.vdf').exists())

    def test_most_recent_login_selects_account(self):
        (self.steam / 'userdata/73/config').mkdir(parents=True)
        (self.steam / 'config').mkdir()
        (self.steam / 'config/loginusers.vdf').write_text(
            '"users" { "76561197960265770" { "MostRecent" "1" } }')
        self.register()
        self.assertFalse((self.steam / 'userdata/73/config/shortcuts.vdf').exists())

    def test_vdf_truncation_and_trailing_data_refused(self):
        blob = self.ns['write_vdf']({'shortcuts': (0, {})})
        with self.assertRaises(ValueError):
            self.ns['read_vdf'](blob[:-1])
        with self.assertRaises(ValueError):
            self.ns['read_vdf'](blob + b'junk')

    def test_artwork_generation_without_external_assets(self):
        subprocess.run([sys.executable, '-c', ART, str(self.state)], check=True)
        for name in ['header', 'capsule', 'hero', 'icon']:
            svg = (self.state / f'artwork/{name}.svg').read_text()
            self.assertIn('<svg', svg)
            self.assertNotIn('<image', svg)
        self.assertTrue((self.state / 'art_project/render.gd').exists())


if __name__ == '__main__':
    unittest.main()
