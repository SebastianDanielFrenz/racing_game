"""Real-process HTTP/restart acceptance; standard library only, no player saves.
python tools/server_smoke.py --exe out/build/relwithdebinfo/server/rg_server.exe --root .
Optional: --source <read-only g2m store> --scope <scope> enables actual tile checks.
"""
import argparse
import json
import pathlib
import subprocess
import tempfile
import time
import urllib.error
import urllib.request


def fetch(url, headers=None):
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers or {}), timeout=90) as response:
        return response.status, dict(response.headers), response.read()


def document(url):
    status, headers, body = fetch(url)
    assert status == 200
    return json.loads(body), headers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', required=True, type=pathlib.Path)
    parser.add_argument('--root', default='.', type=pathlib.Path)
    parser.add_argument('--source', type=pathlib.Path)
    parser.add_argument('--scope', default='home-2026-09r1')
    args = parser.parse_args()
    root, exe = args.root.resolve(), args.exe.resolve()
    with tempfile.TemporaryDirectory(prefix='rg-server-') as temp:
        temp = pathlib.Path(temp)
        config = json.loads((root / 'data/server/server.json').read_text())
        config.update(port=0, state_file=str(temp / 'world.json'), checkpoint_interval_s=0.2)
        # Exercise weather segment reconstruction rather than only an unchanging sky.
        sky = json.loads((root / config['sky']).read_text())
        sky['rate']['default'] = 3600
        sky['rate']['options'].append(3600)
        sky['start']['date'] = '2026-10-22'
        sky_file = temp / 'sky.json'
        sky_file.write_text(json.dumps(sky))
        config['sky'] = str(sky_file)
        schedule = json.loads((root / config['schedule']).read_text())
        schedule['transition_game_s'] = 120
        schedule['min_duration_game_s'] = 900
        schedule['duration_sigma'] = 0
        for state in schedule['states'].values():
            state['mean_hours'] = 0.5
        schedule_file = temp / 'schedule.json'
        schedule_file.write_text(json.dumps(schedule))
        config['schedule'] = str(schedule_file)
        if args.source:
            config['map'].update(enabled=True, source=str(args.source.resolve()), scope=args.scope,
                                 derived=str(temp / 'derived'), port=0, rail=False)
        cfg = temp / 'config.json'
        cfg.write_text(json.dumps(config))

        def start(seconds=3):
            process = subprocess.Popen([str(exe), '--root', str(root), '--config', str(cfg), '--run-for', str(seconds)],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            # Ready is flushed explicitly by the server. Process timeout is enforced below.
            import concurrent.futures
            with concurrent.futures.ThreadPoolExecutor() as pool:
                future = pool.submit(process.stdout.readline)
                try:
                    line = future.result(timeout=90)
                except concurrent.futures.TimeoutError:
                    process.kill()
                    raise AssertionError('server did not announce readiness')
            assert line.startswith('RG_SERVER_READY '), line + process.communicate(timeout=5)[0]
            port = int(line.split('port=')[1].split()[0])
            return process, f'http://127.0.0.1:{port}'

        def finish(process):
            output = process.communicate(timeout=20)[0]
            assert process.returncode == 0, output
            assert 'RG_SERVER_STOP' in output, output

        process, base = start(8 if args.source else 3)
        try:
            health, _ = document(base + '/rg/v1/health')
            assert health['status'] == 'ready'
            info, _ = document(base + '/rg/v1/server')
            assert info['player_replication'] is False
            first, headers = document(base + '/rg/v1/environment')
            assert headers.get('Cache-Control') == 'no-store'
            time.sleep(0.3)
            second, _ = document(base + '/rg/v1/environment')
            assert second['server_us'] > first['server_us']
            elapsed = (second['server_us'] - first['server_us']) / 1e6
            assert abs((second['game_seconds_j2000'] - first['game_seconds_j2000']) - elapsed * 3600) < 1
            assert second['weather']['format'] == 'rg.weather_broadcast/1'
            assert second['weather']['mode'] == 'dynamic'
            # A second process must not corrupt the first world's checkpoint.
            duplicate = subprocess.run([str(exe), '--root', str(root), '--config', str(cfg), '--run-for', '0.1'],
                                       capture_output=True, text=True, timeout=10)
            assert duplicate.returncode != 0 and 'lock' in duplicate.stderr
            try:
                fetch(base + '/rg/v1/environment', {'Content-Type': 'application/json', 'X-Test': 'readonly'})
                urllib.request.urlopen(urllib.request.Request(base + '/rg/v1/environment', data=b'{}'), timeout=5)
                raise AssertionError('environment unexpectedly writable')
            except urllib.error.HTTPError as error:
                assert error.code in (404, 405)
            if args.source:
                maps = info['map']
                map_base = maps['base_url'] + maps['api_path']
                manifest, _ = document(map_base + '/manifest')
                assert manifest['format'] == 'g2m.server/1'
                release, _ = document(map_base + '/releases/' + maps['release_id'] + '/manifest')
                assert release['format'] == 'g2m.release/1'
                # Known terrain sample inside home-r1; other scopes may use another tile.
                tile = map_base + '/releases/' + maps['release_id'] + '/tiles/g2m.terrain.height/utm32n/0/1816/21688'
                status, tile_headers, body = fetch(tile)
                assert status == 200 and len(body) > 100
                try:
                    fetch(tile, {'If-None-Match': tile_headers['ETag']})
                    raise AssertionError('tile validator was ignored')
                except urllib.error.HTTPError as error:
                    assert error.code == 304
        finally:
            finish(process)
        saved = json.loads((temp / 'world.json').read_text())
        assert saved['weather']['change_game_s'] > saved['weather_epoch_game_s'], 'test must cross weather segments'
        time.sleep(0.4)  # Downtime must not advance game time.
        restarted, base = start(1)
        try:
            after, _ = document(base + '/rg/v1/environment')
            assert after['instance_id'] != first['instance_id']
            # Convert local civil clock to compare the saved instant (the run is well within a day).
            assert after['date'] == saved['date']
            assert 0 <= after['local_time_h'] - saved['local_time_h'] < 0.15
            assert after['weather']['change_game_s'] >= saved['weather_epoch_game_s']
            if after['game_seconds_j2000'] < saved['weather']['next_change_game_s']:
                for field in ('mode', 'weather', 'from', 'change_game_s', 'transition_s', 'next_change_game_s'):
                    assert after['weather'][field] == saved['weather'][field], field
        finally:
            finish(restarted)
        original_config = cfg.read_text()
        for changes in ({'port': -1}, {'unexpected': True}):
            invalid = dict(config, **changes)
            cfg.write_text(json.dumps(invalid))
            bad = subprocess.run([str(exe), '--root', str(root), '--config', str(cfg)], capture_output=True, text=True, timeout=10)
            assert bad.returncode != 0 and 'RG_SERVER_ERROR' in bad.stderr
        cfg.write_text(original_config)
        # Malformed state must fail, rather than silently reset a persistent world.
        (temp / 'world.json').write_text('{broken')
        bad = subprocess.run([str(exe), '--root', str(root), '--config', str(cfg)], capture_output=True, text=True, timeout=10)
        assert bad.returncode != 0 and 'RG_SERVER_ERROR' in bad.stderr
        print('RG_SERVER_SMOKE passed: clock, weather, read-only API, exclusive state, restart, corrupt-state rejection'
              + (', tiles, ETag' if args.source else ''))


if __name__ == '__main__':
    main()
