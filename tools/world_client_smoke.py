"""End-to-end world client acceptance using only this harness's own processes."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import tempfile


def ready_line(process, prefix, timeout=45):
    with concurrent.futures.ThreadPoolExecutor() as pool:
        future = pool.submit(process.stdout.readline)
        try:
            line = future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            process.kill()
            raise AssertionError('readiness timed out: ' + prefix)
    if not line.startswith(prefix):
        process.kill()
        raise AssertionError(line + process.communicate()[0])
    return line


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path('.'))
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--scope', default='home-2026-09r1')
    parser.add_argument('--godot', type=Path)
    parser.add_argument('--full-startup', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    binary = root / 'out/build/relwithdebinfo/server'
    suffix = '.exe' if os.name == 'nt' else ''
    env = dict(os.environ)
    with tempfile.TemporaryDirectory(prefix='rg-world-client-') as directory:
        work = Path(directory)
        cfg = json.loads((root / 'data/server/server.json').read_text())
        cfg.update(port=0, state_file=str(work / 'state.json'))
        cfg['map'].update(enabled=True, port=0, source=str(args.source.resolve()), scope=args.scope,
                          derived=str(work / 'derived'), rail=False)
        config = work / 'server.json'
        config.write_text(json.dumps(cfg))
        server = subprocess.Popen([str(binary / ('rg_server' + suffix)), '--root', str(root), '--config', str(config),
                                   '--run-for', '45' if args.full_startup else '14'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        probe = None
        try:
            line = ready_line(server, 'RG_SERVER_READY ')
            port = int(line.split('port=')[1].split()[0])
            url = f'http://127.0.0.1:{port}'
            env['RG_SERVER_URL'] = url
            probe = subprocess.Popen([str(binary / ('rg_world_client_probe' + suffix)), url, str(root), str(work / 'cache')],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            print(ready_line(probe, 'RG_WORLD_CLIENT_READY '), end='')
            if args.godot:
                result = subprocess.run([str(args.godot.resolve()), '--headless', '--path', str(root / 'game'),
                                         '--log-file', str(work / 'godot.log'), '--script', 'res://scripts/world_server_client_test.gd'],
                                        env=env, capture_output=True, text=True, timeout=20)
                assert result.returncode == 0 and 'RG_WORLD_GODOT_PASS' in result.stdout, result.stdout + result.stderr
                print('RG_WORLD_GODOT_PASS authority controls clock weather')
                if args.full_startup:
                    world = json.loads((root / 'data/world/world_config.json').read_text())
                    world['source_store']['dir'] = str(work / 'no-local-source-store')
                    world['derived_store']['dir'] = str(work / 'client-derived')
                    world_file = root / 'out/client_remote_world.json'
                    world_file.write_text(json.dumps(world))
                    result = subprocess.run([str(args.godot.resolve()), '--headless', '--path', str(root / 'game'),
                                             '--log-file', str(root / 'out/client_remote_startup.godot.log'),
                                             '--script', 'res://scripts/terrain_startup_test.gd', '--', str(world_file)],
                                            env=env, capture_output=True, text=True, timeout=65)
                    assert result.returncode == 0 and 'STARTUP_TEST_READY' in result.stdout, result.stdout + result.stderr
                    assert not (work / 'no-local-source-store').exists()
                    print(next(line for line in result.stdout.splitlines() if 'STARTUP_TEST_READY' in line))
            output = server.communicate(timeout=55)[0]
            assert server.returncode == 0, output
            output = probe.communicate('\n', timeout=12)[0]
            assert probe.returncode == 0 and 'RG_WORLD_CLIENT_PASS' in output, output
            print(output.strip())
        finally:
            for process in (probe, server):
                if process is not None and process.poll() is None:
                    process.kill()
                    process.communicate()


if __name__ == '__main__':
    main()
