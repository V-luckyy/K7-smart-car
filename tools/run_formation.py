"""One-command three-car formation run.

Usage:
    python tools/run_formation.py [--shape spiral|figure8] [--key PATH]

What it does (all over SSH, from this PC):
  1. kills any stale launch/logger on the 3 boards
  2. restarts each car (odom resets to its start) with test_mode:=no_perception
  3. starts an odometry logger per car
  4. sends each car its own reference path (ref_path, per-vehicle namespace)
  5. waits for all three to report "path complete"
  6. stops the three tasks, kills the loggers
  7. copies odom CSVs + ref_path logs into logs/formation_<timestamp>/
  8. runs tools/analyze_formation.py to produce formation_plot.png

Environment: the three K7 boards are reachable over SSH; default lab IPs below.
"""
import argparse
import datetime
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CARS = [
    ('car1', '10.148.0.108', 71),
    ('car2', '10.148.0.140', 72),
    ('car3', '10.148.0.113', 73),
]
SETUP = ('source /opt/ros/jazzy/setup.bash; '
         'source $HOME/Code/K7_ros2/install/setup.bash; ')


def ssh(ip, command, key, timeout=120):
    opts = ['-i', key, '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new',
            '-o', 'ConnectTimeout=10']
    return subprocess.run(['ssh'] + opts + [f'kickpi@{ip}', command],
                          capture_output=True, text=True, timeout=timeout)


def scp(ip, local, remote, key):
    opts = ['-i', key, '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new']
    return subprocess.run(['scp'] + opts + [str(local), f'kickpi@{ip}:{remote}'],
                          capture_output=True, text=True, timeout=120)


def fetch(ip, remote, local, key):
    opts = ['-i', key, '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=accept-new']
    return subprocess.run(['scp'] + opts + [f'kickpi@{ip}:{remote}', str(local)],
                          capture_output=True, text=True, timeout=120)


def check(result, what):
    if result.returncode != 0:
        raise RuntimeError(f'{what} failed: {result.stderr.strip() or result.stdout.strip()}')
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shape', default='spiral', choices=['spiral', 'figure8'])
    parser.add_argument('--key', default=os.path.expanduser('~/.ssh/codex_kickpi_ed25519'))
    parser.add_argument('--timeout', type=float, default=900.0)
    args = parser.parse_args()

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = ROOT / 'logs' / f'formation_{stamp}'
    run_dir.mkdir(parents=True)

    print('[1/8] stop stale processes')
    for car, ip, _ in CARS:
        ssh(ip, 'pkill -9 -f "[r]obot.launch.py"; pkill -9 -f "[C]ode/K7_ros2/install"; '
                'pkill -9 -f "[r]os2 run"; pkill -9 -f "[o]dom_logger"; true', args.key)
    time.sleep(2)

    print('[2/8] launch three cars (odom reset)')
    for car, ip, dom in CARS:
        cmd = (f'cd $HOME/Code/K7_ros2; {SETUP} export ROS_DOMAIN_ID={dom}; '
               f'nohup ros2 launch k7_bringup robot.launch.py namespace:={car} '
               f'profile:=trajectory config:=$HOME/k7/{car}.yaml '
               f'test_mode:=no_perception > /tmp/launch_{car}.log 2>&1 & echo ok')
        check(ssh(ip, cmd, args.key), f'launch {car}')

    print('[3/8] wait for task_manager on each car')
    for _ in range(30):
        time.sleep(2)
        ready = True
        for car, ip, dom in CARS:
            out = ssh(ip, f'{SETUP} export ROS_DOMAIN_ID={dom}; ros2 node list 2>/dev/null | grep -c task_manager',
                      args.key).stdout.strip()
            if out not in ('1',):
                ready = False
                break
        if ready:
            break
    else:
        raise RuntimeError('task_manager did not come up on all cars')

    print('[4/8] start odometry loggers')
    logger = ROOT / 'tools' / 'odom_logger.py'
    for car, ip, dom in CARS:
        check(scp(ip, logger, '/tmp/odom_logger.py', args.key), f'upload logger {car}')
        cmd = (f'{SETUP} export ROS_DOMAIN_ID={dom}; '
               f'nohup timeout {int(args.timeout) + 60} python3 /tmp/odom_logger.py {car} '
               f'> /tmp/logger_{car}.log 2>&1 & echo ok')
        check(ssh(ip, cmd, args.key), f'logger {car}')
    time.sleep(2)

    print('[5/8] send reference path to each car')
    for car, ip, dom in CARS:
        cmd = (f'{SETUP} export ROS_DOMAIN_ID={dom}; '
               f'nohup ros2 run k7_tasks ref_path --ros-args -r __ns:=/{car} '
               f'-p shape:={args.shape} > /tmp/ref_path_{car}.log 2>&1 & echo ok')
        check(ssh(ip, cmd, args.key), f'ref_path {car}')

    print('[6/8] wait for completion')
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        time.sleep(5)
        states = []
        for car, ip, dom in CARS:
            out = ssh(ip, f'grep -h "Reference path finished" /tmp/ref_path_{car}.log '
                          f'2>/dev/null | tail -1', args.key).stdout.strip()
            states.append(out)
        print('    ' + ' | '.join(s.split(': ')[-1] if s else 'running' for s in states))
        if all(states):
            break
    else:
        print('    timeout: stopping anyway')

    print('[7/8] stop tasks and loggers')
    for car, ip, dom in CARS:
        ssh(ip, f'{SETUP} export ROS_DOMAIN_ID={dom}; '
                f'ros2 service call /{car}/task/stop std_srvs/srv/Trigger "{{}}"', args.key)
        ssh(ip, 'pkill -f "[o]dom_logger"; true', args.key)

    print('[8/8] fetch data and analyze')
    for car, ip, dom in CARS:
        fetch(ip, f'/tmp/odom_{car}.csv', run_dir / f'odom_{car}.csv', args.key)
        fetch(ip, f'/tmp/ref_path_{car}.log', run_dir / f'ref_path_{car}.log', args.key)
    subprocess.run([sys.executable, str(ROOT / 'tools' / 'analyze_formation.py'),
                    str(run_dir), args.shape], check=False)
    print('done:', run_dir)


if __name__ == '__main__':
    main()
