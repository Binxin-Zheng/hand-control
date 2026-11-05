"""Download the official follower simulation directory at one immutable commit.

No dependencies, installation, camera access, or motor I/O. Run in a network-enabled
shell. Existing differing files are never overwritten. A completion manifest is
written only after every downloaded blob has been checked against its Git SHA.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import urllib.request

REPOSITORY = 'TheRobotStudio/SO-ARM100'
PREFIX = 'Simulation/SO101/'


def fetch(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'hand-control-model-fetch'})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def relative_destination(name):
    if name.startswith(PREFIX):
        relative = name[len(PREFIX):]
    elif name in ('LICENSE', 'LICENSE.md', 'LICENSE.txt'):
        relative = name
    else:
        raise ValueError('Not an approved model or license path')
    p = PurePosixPath(relative)
    if not relative or p.is_absolute() or '..' in p.parts or '\\' in relative:
        raise ValueError('Unsafe upstream path')
    return Path(*p.parts)


def download(output, revision='main'):
    # Resolve branch once: all subsequent reads use this immutable tree/commit.
    from urllib.parse import quote
    commit = json.loads(fetch(f'https://api.github.com/repos/{REPOSITORY}/commits/{quote(revision, safe="")}'))
    sha = commit['sha']
    tree_sha = commit['commit']['tree']['sha']
    tree = json.loads(fetch(f'https://api.github.com/repos/{REPOSITORY}/git/trees/{tree_sha}?recursive=1'))
    if tree.get('truncated'):
        raise RuntimeError('Incomplete Git tree; refusing partial model')
    entries = [entry for entry in tree['tree'] if entry['type'] == 'blob' and
               (entry['path'].startswith(PREFIX) or entry['path'] in ('LICENSE', 'LICENSE.md', 'LICENSE.txt'))]
    names = {entry['path'] for entry in entries}
    for required in ('scene.xml', 'so101_new_calib.xml', 'so101_new_calib.urdf'):
        if PREFIX + required not in names:
            raise RuntimeError(f'Official follower model missing {required}')
    manifest = {'repository': REPOSITORY, 'commit': sha, 'files': {}}
    for entry in entries:
        name = entry['path']
        target = output / relative_destination(name)
        data = fetch(f'https://raw.githubusercontent.com/{REPOSITORY}/{sha}/{quote(name, safe="/")}')
        git_hash = hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
        if git_hash != entry['sha']:
            raise RuntimeError(f'Git blob mismatch: {name}')
        if target.exists() and target.read_bytes() != data:
            raise FileExistsError(f'Refusing to overwrite different file: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        manifest['files'][str(relative_destination(name))] = {
            'git_blob': git_hash, 'sha256': hashlib.sha256(data).hexdigest()}
    (output / 'source-manifest.json').write_text(json.dumps(manifest, indent=2))
    print(f'Verified {len(entries)} files from {REPOSITORY}@{sha} into {output}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('assets/so101'))
    parser.add_argument('--revision', default='main')
    args = parser.parse_args()
    download(args.out, args.revision)
