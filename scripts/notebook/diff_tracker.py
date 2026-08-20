"""Tracks which git commit the notebook system last drafted from."""

import subprocess
from pathlib import Path


def get_last_entry(state_path: str) -> str | None:
    """Return the stored commit hash, or None if no entry has run yet."""
    path = Path(state_path)
    if not path.exists():
        return None
    return path.read_text().strip() or None


def update_last_entry(state_path: str, commit_hash: str) -> None:
    """Persist the commit hash the notebook was last drafted through."""
    path = Path(state_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(commit_hash + "\n")


def get_current_commit(repo_root: str) -> str:
    """Return HEAD's full commit hash in repo_root."""
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def get_diff_since(repo_root: str, since_commit: str | None) -> str:
    """Return the unified diff since since_commit.

    If since_commit is None (first-ever run), return only the diff
    introduced by the current HEAD commit, not the whole repo history -
    a first run should draft one page from the latest change, not the
    entire project's past.
    """
    if since_commit is None:
        cmd = ["git", "show", "HEAD"]
    else:
        cmd = ["git", "diff", since_commit, "HEAD"]
    result = subprocess.run(
        cmd,
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout
