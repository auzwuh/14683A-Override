import subprocess
import tempfile
from pathlib import Path

import pytest

from diff_tracker import (
    get_last_entry,
    get_diff_since,
    get_current_commit,
    update_last_entry,
)


@pytest.fixture
def tmp_repo():
    with tempfile.TemporaryDirectory() as tmp:
        repo = Path(tmp)
        subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
        subprocess.run(["git", "config", "user.email", "test@test.com"], cwd=repo, check=True)
        subprocess.run(["git", "config", "user.name", "Test"], cwd=repo, check=True)
        (repo / "a.txt").write_text("one\n")
        subprocess.run(["git", "add", "a.txt"], cwd=repo, check=True)
        subprocess.run(["git", "commit", "-q", "-m", "first"], cwd=repo, check=True)
        yield repo


def test_get_last_entry_missing_file_returns_none(tmp_repo):
    state_path = tmp_repo / "docs" / "notebook" / ".last_entry"
    assert get_last_entry(str(state_path)) is None


def test_update_then_get_last_entry_roundtrips(tmp_repo):
    state_path = tmp_repo / "docs" / "notebook" / ".last_entry"
    update_last_entry(str(state_path), "abc123")
    assert get_last_entry(str(state_path)) == "abc123"
    assert state_path.exists()


def test_get_current_commit_matches_git_rev_parse(tmp_repo):
    expected = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=tmp_repo, check=True, capture_output=True, text=True
    ).stdout.strip()
    assert get_current_commit(str(tmp_repo)) == expected


def test_get_diff_since_none_shows_only_head_commit(tmp_repo):
    diff = get_diff_since(str(tmp_repo), None)
    assert "a.txt" in diff
    assert "+one" in diff


def test_get_diff_since_commit_shows_only_later_changes(tmp_repo):
    first_commit = get_current_commit(str(tmp_repo))
    (tmp_repo / "b.txt").write_text("two\n")
    subprocess.run(["git", "add", "b.txt"], cwd=tmp_repo, check=True)
    subprocess.run(["git", "commit", "-q", "-m", "second"], cwd=tmp_repo, check=True)

    diff = get_diff_since(str(tmp_repo), first_commit)
    assert "b.txt" in diff
    assert "a.txt" not in diff
