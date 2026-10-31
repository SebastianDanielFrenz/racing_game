#!/usr/bin/env python3
"""Create or update a non-Steam shortcut by writing Steam's shortcuts.vdf.

Used by tools/frame_install.ps1 on the Steam Frame, where Steam always runs as
the systemd user service steam.service. Steam keeps shortcuts.vdf in memory and
rewrites it on exit, so the file is only changed while Steam is stopped:

  * nothing to change      -> Steam is not touched (shortcut=unchanged);
  * Steam not running      -> the file is written directly;
  * Steam running          -> only with --restart-steam: `systemctl --user stop
                              steam.service`, re-read the file Steam wrote on
                              exit, write, `systemctl --user start steam.service`.
                              Without the flag nothing is written.

An existing entry is matched by its Exe, or by a SteamOS devkit entry with the
same DevkitGameID (that one is converted into a plain shortcut, so the game
does not show up twice). The binary KeyValues reader/writer preserves every
field and type it does not know, refuses anything it cannot round-trip, keeps
a timestamped backup and writes atomically. Standard library only.

The VDF reader/writer is the same code as the one embedded in
tools/install_linux.sh (that script has to stay a single downloadable file).
"""
import argparse, os, pathlib, re, shutil, struct, subprocess, sys, tempfile, time, zlib


def read_vdf(data):
    pos = 0

    def string():
        nonlocal pos
        end = data.index(b'\0', pos)
        result = data[pos:end].decode('utf-8')
        pos = end + 1
        return result

    def obj():
        nonlocal pos
        result = {}
        while pos < len(data):
            kind = data[pos]
            pos += 1
            if kind == 8:
                return result
            key = string()
            if key in result:
                raise ValueError('Duplicate VDF key; refusing lossy rewrite')
            if kind == 0:
                value = obj()
            elif kind == 1:
                value = string()
            elif kind in (2, 3, 4, 6, 7, 10):
                size = 8 if kind in (7, 10) else 4
                value = data[pos:pos + size]
                if len(value) != size:
                    raise ValueError('Truncated VDF value')
                pos += size
            else:
                raise ValueError(f'Unsupported VDF type {kind}; original file unchanged')
            result[key] = (kind, value)
        raise ValueError('Unterminated VDF object')

    result = obj()
    if pos != len(data):
        raise ValueError('Trailing VDF data; original file unchanged')
    return result


def write_vdf(obj):
    result = bytearray()
    for key, (kind, value) in obj.items():
        result += bytes([kind]) + key.encode() + b'\0'
        result += write_vdf(value) if kind == 0 else value.encode() + b'\0' if kind == 1 else value
    return bytes(result) + b'\x08'


def atomically_write(path, data):
    fd, temp = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def steam_running():
    for proc in pathlib.Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if proc.stat().st_uid == os.getuid() and (proc / 'comm').read_text().strip() in ('steam', 'steam.exe'):
                return True
        except (OSError, PermissionError):
            pass
    return False


def find_user_config(user):
    home = pathlib.Path.home()
    candidates = [home / '.local/share/Steam', home / '.steam/steam', home / '.steam/root']
    steam = next((p.resolve() for p in candidates if (p / 'userdata').is_dir()), None)
    if steam is None:
        raise SystemExit('shortcut=error Steam userdata not found (log in to Steam once)')
    users = [p for p in (steam / 'userdata').iterdir() if p.is_dir() and p.name.isdigit() and p.name != '0']
    if user:
        users = [p for p in users if p.name == user]
    elif len(users) > 1:
        login = steam / 'config/loginusers.vdf'
        if login.exists():
            latest = [sid for sid, body in re.findall(r'"(\d{17})"\s*\{([^{}]*)\}', login.read_text())
                      if re.search(r'"MostRecent"\s*"1"', body, re.I)]
            if len(latest) == 1:
                users = [p for p in users if int(p.name) == int(latest[0]) - 76561197960265728]
    if len(users) != 1:
        raise SystemExit('shortcut=error cannot pick the Steam account; pass --user, candidates: '
                         + ', '.join(p.name for p in users))
    config = users[0] / 'config'
    config.mkdir(exist_ok=True)
    return users[0].name, config / 'shortcuts.vdf'


def u32(value):
    return (2, struct.pack('<I', value))


def plan(path, args):
    """Returns (original bytes or None, new bytes, action, appid)."""
    original = path.read_bytes() if path.exists() else None
    tree = read_vdf(original) if original else {'shortcuts': (0, {})}
    root_key = next((key for key in tree if key.lower() == 'shortcuts'), None)
    if root_key is None or tree[root_key][0] != 0:
        raise ValueError('Invalid shortcuts root')
    entries = tree[root_key][1]
    exe = '"' + args.exe + '"'

    def field(entry, name):
        return next((v for k, v in entry.items() if k.lower() == name.lower()), None)

    def set_field(entry, name, value):
        actual = next((k for k in entry if k.lower() == name.lower()), name)
        entry[actual] = value

    def is_devkit_match(entry):
        gid = field(entry, 'DevkitGameID')
        return bool(args.devkit_gameid) and gid is not None and gid[0] == 1 and gid[1] == args.devkit_gameid

    existing = next((e for kind, e in entries.values() if kind == 0 and field(e, 'Exe') == (1, exe)), None)
    if existing is None:
        existing = next((e for kind, e in entries.values() if kind == 0 and is_devkit_match(e)), None)
    used = {struct.unpack('<I', v[1])[0] for kind, e in entries.values()
            if kind == 0 and (v := field(e, 'appid')) and v[0] == 2}
    if existing is not None and field(existing, 'appid') and field(existing, 'appid')[0] == 2:
        appid = struct.unpack('<I', field(existing, 'appid')[1])[0]
    else:
        appid = zlib.crc32((exe + args.title).encode()) | 0x80000000
        while appid in used:
            appid = ((appid + 1) & 0xffffffff) | 0x80000000
    values = {'appid': u32(appid), 'AppName': (1, args.title), 'Exe': (1, exe),
              'StartDir': (1, '"' + args.start_dir + '"'), 'icon': (1, args.icon or ''),
              'LaunchOptions': (1, ''), 'IsHidden': u32(0), 'AllowDesktopConfig': u32(1),
              'AllowOverlay': u32(1), 'OpenVR': u32(int(args.openvr)), 'Devkit': u32(0),
              'DevkitGameID': (1, ''), 'tags': (0, {})}
    if existing is not None:
        # Keep user choices (title, launch options, tags, AppID); fix what this
        # script owns. A devkit entry becomes a plain shortcut.
        owned = ['Exe', 'StartDir', 'OpenVR'] + (['icon'] if args.icon else [])
        if field(existing, 'Devkit') not in (None, u32(0)):
            owned += ['Devkit', 'DevkitGameID', 'AppName']
        for key in owned:
            set_field(existing, key, values[key])
        action = 'updated'
    else:
        index = 0
        while str(index) in entries:
            index += 1
        entries[str(index)] = (0, values)
        action = 'created'
    packed = write_vdf(tree)
    if read_vdf(packed) != tree:
        raise ValueError('VDF roundtrip failed')
    if original == packed:
        action = 'unchanged'
    return original, packed, action, appid


def systemctl(*words):
    return subprocess.run(['systemctl', '--user', *words], stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True).returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--exe', required=True, help='absolute path of the launch script')
    ap.add_argument('--start-dir', required=True)
    ap.add_argument('--title', required=True)
    ap.add_argument('--icon', default='')
    ap.add_argument('--openvr', type=int, choices=(0, 1), default=0)
    ap.add_argument('--devkit-gameid', default='', help='also adopt a SteamOS devkit entry with this game id')
    ap.add_argument('--user', default='', help='numeric Steam userdata account id')
    ap.add_argument('--restart-steam', action='store_true',
                    help='stop and restart steam.service if Steam is running and the file must change')
    args = ap.parse_args()

    account, path = find_user_config(args.user)
    _, packed, action, appid = plan(path, args)
    if action == 'unchanged':
        print(f'shortcut=unchanged appid={appid} account={account}')
        return
    restarted = False
    if steam_running():
        if not args.restart_steam:
            raise SystemExit(f'shortcut=steam-running ({action} needed; rerun with --restart-steam or exit Steam)')
        print('steam: stopping steam.service to write shortcuts.vdf')
        systemctl('stop', 'steam.service')
        for _ in range(60):
            if not steam_running():
                break
            time.sleep(0.5)
        if steam_running():
            systemctl('start', 'steam.service')
            raise SystemExit('shortcut=error Steam did not exit within 30 s; nothing written')
        restarted = True
        # Steam rewrites the file on exit: plan again on what it left behind.
        _, packed, action, appid = plan(path, args)
    try:
        if action != 'unchanged':
            if path.exists():
                shutil.copy2(path, path.with_name(path.name + '.racing-game-' + str(time.time_ns()) + '.bak'))
            atomically_write(path, packed)
    finally:
        if restarted:
            rc = systemctl('start', 'steam.service')
            print(f'steam: steam.service started again (rc={rc})')
    print(f'shortcut={action} appid={appid} account={account}')


if __name__ == '__main__':
    try:
        main()
    except ValueError as e:
        raise SystemExit(f'shortcut=error {e}; shortcuts.vdf unchanged')
