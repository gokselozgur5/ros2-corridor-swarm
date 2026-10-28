"""steps/01_init.sh and steps/04_run.sh are the day-1 scripts (ops-scripts-5).

Run again on an initialised checkout they used to rewrite .gitignore, env.sh and ws/deps.repos and then
`git add -A`, which on Thor would commit .env with the live key and the vcs-imported repos as gitlinks.
Each script runs here in a scratch checkout that holds the worktree's current versions of those files,
a dummy .env and one commit; it has to refuse before writing or committing anything.
"""
import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
STEPS = REPO / 'steps'
DAY1 = ('01_init.sh', '04_run.sh')
TRACKED = ('.gitignore', 'env.sh', 'ws/deps.repos')
# the day-1 scripts cd here before anything else; the scratch checkout sits at the same place under the
# test's HOME so that a script which ignores CORRIDOR_ROOT still reaches its writes instead of dying on cd
HARDCODED = 'deletdis/ros2-corridor-swarm'


def _executable(path, body):
    path.write_text(body)
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def _git(env, root, *args):
    return subprocess.run(['git', '-C', str(root), *args], env=env, check=True, capture_output=True, text=True).stdout


def _environment(tmp_path, root):
    """HOME under tmp_path with a stub ROS setup.bash, stub ros2/tmux/vcs first on PATH, CORRIDOR_ROOT at root."""
    home = tmp_path / 'home'
    setup = home / '.goske/ros2_jazzy/install/setup.bash'
    setup.parent.mkdir(parents=True, exist_ok=True)
    setup.write_text('export ROS_DISTRO=jazzy-stub\n')
    stubs = tmp_path / 'stubs'
    stubs.mkdir()
    for name in ('ros2', 'tmux', 'vcs'):
        _executable(stubs / name, f'#!/bin/sh\necho "stub {name} $*"\nexit 0\n')
    env = {k: v for k, v in os.environ.items() if k != 'XDG_CONFIG_HOME'}
    env.update(HOME=str(home), CORRIDOR_ROOT=str(root), PATH=f'{stubs}:{env["PATH"]}', GIT_CONFIG_NOSYSTEM='1')
    return env


def _initialised_checkout(env, root):
    """A git repo with the worktree's .gitignore, env.sh and ws/deps.repos committed and a dummy .env beside them."""
    root.mkdir(parents=True)
    for rel in TRACKED:
        (root / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(REPO / rel, root / rel)
    (root / '.env').write_text('ANTHROPIC_API_KEY=dummy\n')
    _git(env, root, 'init', '-q')
    _git(env, root, 'config', 'user.name', 'test')
    _git(env, root, 'config', 'user.email', 'test@example.invalid')
    _git(env, root, 'add', *TRACKED)
    _git(env, root, 'commit', '-q', '-m', 'initialised checkout')
    return root


def _run(script, env, cwd):
    return subprocess.run(['bash', str(STEPS / script)], env=env, cwd=cwd, capture_output=True, text=True, timeout=120)


@pytest.mark.parametrize('script', DAY1)
def test_day1_script_refuses_an_initialised_checkout(tmp_path, script):
    root = tmp_path / 'home' / HARDCODED
    env = _environment(tmp_path, root)
    _initialised_checkout(env, root)
    before = {rel: (root / rel).read_bytes() for rel in TRACKED}
    commits = _git(env, root, 'rev-list', '--count', 'HEAD')

    run = _run(script, env, tmp_path)

    violations = []
    if run.returncode == 0:
        violations.append('exit code 0')
    if 'already exists' not in run.stderr:
        violations.append(f'no refusal line on stderr: {run.stderr.strip()!r}')
    violations += [f'{rel} rewritten' for rel in TRACKED if (root / rel).read_bytes() != before[rel]]
    if _git(env, root, 'rev-list', '--count', 'HEAD') != commits:
        violations.append('a new commit was made')
    tracked = subprocess.run(['git', '-C', str(root), 'ls-files', '--error-unmatch', '.env'], env=env, capture_output=True)
    if tracked.returncode == 0:
        violations.append('.env is tracked')
    assert violations == []


@pytest.mark.parametrize('script', DAY1)
def test_day1_script_takes_the_checkout_from_corridor_root(tmp_path, script):
    root = tmp_path / 'elsewhere'
    env = _environment(tmp_path, root)
    _initialised_checkout(env, root)
    assert not (tmp_path / 'home' / HARDCODED).exists()

    run = _run(script, env, tmp_path)

    assert run.returncode != 0 and 'already exists' in run.stderr, run.stderr


@pytest.mark.parametrize('script', DAY1)
def test_day1_script_stages_its_own_files_not_everything(script):
    assert 'git add -A' not in (STEPS / script).read_text()
