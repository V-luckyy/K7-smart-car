"""Build the architecture reader from checked Archify diagrams and interface sources."""
import argparse
from copy import deepcopy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'docs/k7-architecture.json'
TEMPLATE = ROOT / 'docs/k7-architecture.template.html'


def write_candidates(suite, directory):
    for diagram in suite['diagrams']:
        folder = directory / diagram['id']
        folder.mkdir(parents=True, exist_ok=True)
        spec = deepcopy(diagram['spec'])
        spec['meta']['output'] = (folder / (diagram['id'] + '.html')).relative_to(ROOT).as_posix()
        (folder / 'candidate.json').write_text(json.dumps(spec, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def assemble(suite, directory):
    payload = deepcopy(suite)
    revision = suite['repository']['revision']
    for diagram in payload['diagrams']:
        folder = directory / diagram['id']
        artifact = folder / (diagram['id'] + '.html')
        # The delivery receipt binds the exact embedded bytes, including after repairs.
        delivery = json.loads(artifact.with_suffix('.delivery.json').read_text())
        content = artifact.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        if delivery['status'] != 'current' or delivery['artifact']['sha256'] != digest:
            raise ValueError(f"{diagram['id']} artifact differs from delivery receipt")
        candidate_digest = hashlib.sha256((folder / 'candidate.json').read_bytes()).hexdigest()
        summaries = [json.loads(path.read_text()) for path in folder.rglob('*.finalize-summary.json')]
        if not any(summary.get('status') == 'pass'
                   and summary['artifact']['sha256'] == digest
                   and summary['specification']['sha256'] == candidate_digest
                   and all(value == 'pass' for value in summary['gates'].values())
                   for summary in summaries):
            raise ValueError(f"{diagram['id']} has no matching passing finalize receipt")
        diagram['html'] = content.decode('utf-8')
        del diagram['spec']
    for interface in payload['interfaces']:
        if interface.get('definition'):
            interface['raw_definition'] = subprocess.check_output(
                ['git', 'show', revision + ':' + interface['definition']], cwd=ROOT, text=True)
    serialized = json.dumps(payload, ensure_ascii=False).replace('<', '\\u003c')
    template = TEMPLATE.read_text(encoding='utf-8')
    assert template.count('__ARCHITECTURE_DATA__') == 1
    output = ROOT / 'docs/k7-architecture.html'
    output.write_text(template.replace('__ARCHITECTURE_DATA__', serialized), encoding='utf-8')
    print(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir', type=Path, help='Relative .archify request directory; reuse for repairs')
    parser.add_argument('--archify', type=Path, help='Installed bin/archify.mjs for full rebuild')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--prepare', action='store_true', help='Write complete candidates only')
    modes.add_argument('--assemble', action='store_true', help='Package already finalized artifacts')
    args = parser.parse_args()
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d-%H%M%S')
    directory = (ROOT / (args.work_dir or Path('.archify/architecture-k7-' + stamp))).resolve()
    if not directory.is_relative_to(ROOT / '.archify'):
        parser.error('--work-dir must be within the repository .archify directory')
    suite = json.loads(SOURCE.read_text(encoding='utf-8'))
    ids = [item['id'] for item in suite['interfaces']]
    assert len(ids) == len(set(ids))
    if not args.assemble:
        write_candidates(suite, directory)
    if args.prepare:
        print(directory)
        return
    if not args.assemble:
        if not args.archify:
            parser.error('--archify is required for a full rebuild')
        for diagram in suite['diagrams']:
            folder = directory / diagram['id']
            artifact = folder / (diagram['id'] + '.html')
            command = ['node', str(args.archify.resolve()), 'finalize', diagram['spec']['diagram_type'],
                       str(folder / 'candidate.json'), str(artifact), '--repo-root', str(ROOT),
                       '--quality', 'showcase', '--json']
            # New evidence slots preserve receipts from any earlier finalized candidate.
            if artifact.exists():
                command.extend(['--out-dir', str(folder / ('review-' + stamp))])
            subprocess.run(command, cwd=ROOT, check=True)
    assemble(suite, directory)


if __name__ == '__main__':
    main()
