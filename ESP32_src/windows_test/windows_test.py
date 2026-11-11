"""Real USB -> leader ESP32 -> ESP-NOW -> follower ESP32 -> USB test."""
import argparse
from collections import OrderedDict
from dataclasses import asdict
import importlib
import json
from pathlib import Path
import secrets
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
# The downloadable bundle contains a generated copy of the source protocol.
# The repository always imports the canonical source so both cannot diverge.
PROJECT = HERE.parents[1]
SOURCE = PROJECT / 'ROS2_src/K7_ros2/src/k7_tasks'
sys.path.insert(0, str(SOURCE if SOURCE.is_dir() else HERE))
if SOURCE.is_dir():
    from k7_tasks.espnow_protocol import FleetPacket, StatusPacket, StreamParser, encode_fleet
else:
    from espnow_protocol import FleetPacket, StatusPacket, StreamParser, encode_fleet


def serial_dependency():
    try:
        return importlib.import_module('serial')
    except ModuleNotFoundError:
        target = HERE / '.runtime/site-packages'
        sys.path.insert(0, str(target))
        try:
            return importlib.import_module('serial')
        except ModuleNotFoundError:
            print('首次运行：安装 pyserial 3.5 到脚本本地目录……', flush=True)
            subprocess.run([sys.executable, '-m', 'pip', 'install', '--target', str(target),
                            'pyserial==3.5'], check=True)
            importlib.invalidate_caches()
            return importlib.import_module('serial')


class Board:
    def __init__(self, serial_module, device):
        self.port = serial_module.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.1)
        self.port.dtr = False
        self.port.rts = False
        self.port.port = device
        self.port.open()
        self.parser = StreamParser()
        self.status = None
        self.status_time = 0.0

    def read(self):
        packets = self.parser.feed(self.port.read(min(self.port.in_waiting, 4096)))
        for packet in packets:
            if isinstance(packet, StatusPacket):
                if self.status is not None and (packet.mac, packet.sender_id, packet.role,
                        packet.channel) != (self.status.mac, self.status.sender_id,
                                           self.status.role, self.status.channel):
                    raise RuntimeError('测试期间板卡身份/配置改变，请重新识别设备')
                self.status, self.status_time = packet, time.monotonic()
        return packets

    def close(self):
        self.port.close()


def discover(serial_module, requested, timeout):
    from serial.tools import list_ports
    devices = requested or [p.device for p in list_ports.comports() if p.vid == 0x1a86]
    if not devices:
        raise RuntimeError('未发现 CH340 串口。请连接两块板、检查数据线和驱动，或用 --ports 指定端口')
    print('检查串口：' + ', '.join(devices), flush=True)
    boards = []
    try:
        for device in devices:
            try:
                boards.append(Board(serial_module, device))
            except (OSError, serial_module.SerialException) as exc:
                print(f'{device} 打开失败：{exc}', flush=True)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for board in boards:
                board.read()
            if boards and all(board.status for board in boards):
                break
            time.sleep(0.02)
        leaders = [b for b in boards if b.status and b.status.sender_id == 1 and b.status.role == 1]
        followers = [b for b in boards if b.status and b.status.sender_id in (2, 3) and b.status.role == 2]
        if len(leaders) != 1 or len(followers) != 1:
            found = ', '.join(f'{b.port.port}:car{b.status.sender_id}/role{b.status.role}'
                              if b.status else f'{b.port.port}:无固件状态' for b in boards)
            raise RuntimeError('请连接一块主车和一块从车，并关闭其他串口程序。当前：' + found)
        leader, follower = leaders[0], followers[0]
        if leader.status.channel != follower.status.channel:
            raise RuntimeError('两块固件的无线信道不同，需要修正固件配置')
        for board in list(boards):
            if board not in (leader, follower):
                board.close()
                boards.remove(board)
        for board in boards:
            status = board.status
            mac = ':'.join(f'{v:02X}' for v in status.mac)
            print(f'已识别 car{status.sender_id}：{board.port.port}，MAC={mac}，信道={status.channel}')
        return leader, follower
    except BaseException:
        for board in boards:
            board.close()
        raise


class Verification:
    def __init__(self):
        self.sent = OrderedDict()
        self.seen = set()
        self.last_sequence = -1
        self.counts = dict(sent=0, received=0, duplicate=0, out_of_order=0,
                           mismatched=0, unexpected=0, cached_during_pause=0)
        self.received_by_phase = dict(stream=0, pause=0, recovery=0)
        self.latencies_ms = []
        self.gaps_ms = []
        self.last_receive = None
        self.last_phase = None

    def remember(self, packet, data, now):
        self.sent[packet.sequence] = (packet.boot_id, packet.session_id, data, now)
        if len(self.sent) > 4096:
            del self.sent[next(iter(self.sent))]
        self.counts['sent'] += 1

    def receive(self, packet, now, phase, phase_elapsed):
        expected = self.sent.get(packet.sequence)
        if expected is None or (packet.boot_id, packet.session_id) != expected[:2]:
            self.counts['unexpected'] += 1
            return
        if encode_fleet(packet) != expected[2]:
            self.counts['mismatched'] += 1
            return
        if phase == 'pause' and phase_elapsed >= 1.0:
            self.counts['cached_during_pause'] += 1
        if packet.sequence in self.seen:
            self.counts['duplicate'] += 1
            return
        self.seen.add(packet.sequence)
        if packet.sequence <= self.last_sequence:
            self.counts['out_of_order'] += 1
        self.last_sequence = max(packet.sequence, self.last_sequence)
        self.counts['received'] += 1
        self.received_by_phase[phase] += 1
        self.latencies_ms.append((now - expected[3]) * 1000)
        if self.last_receive is not None and phase == self.last_phase and phase != 'pause':
            self.gaps_ms.append((now - self.last_receive) * 1000)
        self.last_receive, self.last_phase = now, phase

    def result(self):
        c = self.counts
        ok = (self.received_by_phase['stream'] >= 10 and self.received_by_phase['recovery'] >= 5
              and c['received'] / max(1, c['sent']) >= 0.8
              and max(self.gaps_ms, default=0) <= 500
              and all(c[name] == 0 for name in ('duplicate', 'out_of_order', 'mismatched',
                                               'unexpected', 'cached_during_pause')))
        return dict(passed=ok, counts=dict(c), received_by_phase=dict(self.received_by_phase),
            missing=c['sent'] - c['received'], receive_ratio=c['received'] / max(1, c['sent']),
            latency_mean_ms=sum(self.latencies_ms) / max(1, len(self.latencies_ms)),
            latency_max_ms=max(self.latencies_ms, default=0),
            max_receive_gap_ms=max(self.gaps_ms, default=0))


def test_pair(leader, follower, args, output):
    verification = Verification()
    boot_id, session_id = secrets.randbits(64) or 1, uuid.uuid4().bytes
    sequence = 0
    started = time.monotonic()
    next_send, next_print = started, started
    phases = [('stream', args.duration), ('pause', 3.0), ('recovery', 10.0)]
    status_start = [asdict(b.status) for b in (leader, follower)]
    interrupted = True
    with (output / 'frames.jsonl').open('w', encoding='utf-8') as log:
        try:
            for phase, duration in phases:
                phase_start = time.monotonic()
                next_send = phase_start
                print(f'\n阶段 {phase}，持续 {duration:g} 秒', flush=True)
                while time.monotonic() - phase_start < duration:
                    now = time.monotonic()
                    if phase != 'pause' and now >= next_send:
                        stamp = time.time_ns()
                        # Synthetic fleet-frame metres/radians and body-frame m/s, rad/s.
                        packet = FleetPacket(1, boot_id, sequence, stamp // 10**9, stamp % 10**9,
                            session_id, sequence * 0.001, -0.25, 0.125, 0.05, -0.025, 0, 'fleet')
                        data = encode_fleet(packet)
                        if leader.port.write(data) != len(data):
                            raise RuntimeError('主车 USB 写入不完整')
                        verification.remember(packet, data, now)
                        log.write(json.dumps(dict(event='tx', t=now-started, phase=phase,
                                                  hex=data.hex())) + '\n')
                        sequence += 1
                        next_send = now + 0.1
                    for board in (leader, follower):
                        for packet in board.read():
                            if isinstance(packet, FleetPacket) and board is follower:
                                verification.receive(packet, time.monotonic(), phase,
                                                     time.monotonic() - phase_start)
                                log.write(json.dumps(dict(event='rx', t=time.monotonic()-started,
                                    phase=phase, hex=encode_fleet(packet).hex())) + '\n')
                        if now - board.status_time > 3.0:
                            raise RuntimeError(f'{board.port.port} 固件状态超时，检查 USB 连接')
                    if phase != 'pause' and now - phase_start > 5.0 and (
                            verification.last_receive is None or now - verification.last_receive > 5.0):
                        raise RuntimeError('从车持续 5 秒未收到有效状态，核对主车 MAC、天线和固件信道')
                    if now >= next_print:
                        c = verification.counts
                        print(f"发送={c['sent']} 接收={c['received']} 内容错误={c['mismatched']} "
                              f"无线收包计数={follower.status.radio_rx}", flush=True)
                        next_print = now + 2.0
                    time.sleep(0.005)
            interrupted = False
        finally:
            result = verification.result()
            result.update(interrupted=interrupted, source='synthetic Windows USB test',
                          boards_start=status_start,
                          boards_end=[asdict(b.status) for b in (leader, follower)],
                          parser_errors=[b.parser.bad_frames for b in (leader, follower)])
            result['passed'] = result['passed'] and not interrupted and not any(result['parser_errors'])
            (output / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False,
                default=lambda value: value.hex() if isinstance(value, bytes) else str(value)), encoding='utf-8')
            print('\n' + ('测试通过' if result['passed'] else '测试未通过或被中断'), flush=True)
            print(json.dumps({key: value for key, value in result.items() if not key.startswith('boards')},
                             ensure_ascii=False, indent=2))
            print('日志：' + str(output), flush=True)
    return result['passed']


def main():
    parser = argparse.ArgumentParser(description='一主一从 ESP-NOW 实际通信测试，无需 ROS/K7')
    parser.add_argument('--ports', nargs='+', help='手动指定，例如 --ports COM3 COM7；默认识别 CH340')
    parser.add_argument('--duration', type=float, default=60, help='首段发送时长，默认 60 秒')
    parser.add_argument('--once', action='store_true', help='完成一组即退出；默认可换从车再测')
    args = parser.parse_args()
    if not 2 <= args.duration <= 3600:
        parser.error('--duration 范围为 2..3600 秒')
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    serial_module = serial_dependency()
    print('请连接 car1 和 car2（或 car3），关闭 Arduino 串口监视器。')
    print('测试数据仅经 USB/ESP-NOW 传输；本工具不连接 ROS 或底盘。', flush=True)
    while True:
        boards = []
        passed = False
        try:
            leader, follower = discover(serial_module, args.ports, 10)
            boards = [leader, follower]
            output = HERE / 'results' / (time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6])
            output.mkdir(parents=True)
            passed = test_pair(leader, follower, args, output)
        except (OSError, RuntimeError, serial_module.SerialException) as exc:
            print('检查失败：' + str(exc), file=sys.stderr, flush=True)
            print('若从车有本板状态却无车辆帧，请核对从车 LEADER_MAC 与主车 STA MAC。')
        finally:
            for board in boards:
                board.close()
        if args.once:
            return 0 if passed else 1
        reply = input('\n保留主车，换另一块从车后按回车重测；输入 q 退出：').strip().lower()
        if reply == 'q':
            return 0 if passed else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\n已停止，串口已释放。')
        raise SystemExit(130)
