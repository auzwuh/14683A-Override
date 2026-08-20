# Notebook Authoring — Phase A (Draft) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the diff-tracking, code-snippet-rendering, and category-classification pieces of the engineering-notebook authoring system that have zero dependency on the (currently unauthorized) Canva plugin, so drafting is usable today.

**Architecture:** Three small, independently-testable Python scripts (diff tracker, snippet renderer, and the classification reference data they both feed) wired together by one Claude Code skill (`.claude/skills/notebook/SKILL.md`) that the user invokes on demand. The skill is a playbook, not unit-testable code — it's verified by an end-to-end dry run against this repo's real current diff, not pytest.

**Tech Stack:** Python 3.12 (already installed at `C:\Users\USER\AppData\Local\Programs\Python\Python312\python.exe`), `pytest` + `pygments` (installed in Task 1), git (already used throughout this repo's tooling), the existing Browser pane tool for HTML-to-image rasterization (no new rendering dependency).

**Spec:** `docs/superpowers/specs/2026-08-17-engineering-notebook-authoring-design.md`

## Global Constraints

- Phase A only. Phase B (Canva placement via `canva:edit-design`) is a separate plan, written only after the user authorizes the Canva plugin — per the spec, Phase A has zero Canva dependency and must stay that way.
- Trigger is on-demand only (spec's explicit non-goal: no automatic/passive triggering).
- Diff-tracker state lives at `docs/notebook/.last_entry`, holding a single git commit hash.
- Code figures are rendered, syntax-highlighted images, never literal screen captures (spec + brainstorming decision).
- Category classification is one of exactly 8 values: Strategies, Goals/Plans, Research/Brainstorm, Prototypes, Experiment, Analysis/Discussions, Build/Code, Other — taken verbatim from the 78181A Genesis reference notebook's own legend page.

---

### Task 1: Diff tracker

**Files:**
- Create: `scripts/notebook/diff_tracker.py`
- Create: `scripts/notebook/requirements.txt`
- Test: `scripts/notebook/test_diff_tracker.py`

**Interfaces:**
- Consumes: nothing (first task; only git and the filesystem)
- Produces:
  - `get_last_entry(state_path: str) -> str | None` — returns the stored commit hash, or `None` if the state file doesn't exist yet (first-ever run).
  - `get_diff_since(repo_root: str, since_commit: str | None) -> str` — returns the unified diff text. When `since_commit` is `None`, returns the diff for just the current `HEAD` commit (`git show`), not the entire repo history — a first run should not try to draft the whole project's history into one page.
  - `get_current_commit(repo_root: str) -> str` — returns `HEAD`'s full commit hash.
  - `update_last_entry(state_path: str, commit_hash: str) -> None` — writes the hash, creating parent directories if needed.

- [ ] **Step 1: Set up the scripts directory and dependencies**

```bash
mkdir -p "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook"
```

Create `scripts/notebook/requirements.txt`:
```
pytest>=8.0
pygments>=2.17
```

Install:
```bash
"/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -m pip install -r "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook/requirements.txt"
```

- [ ] **Step 2: Write the failing tests**

Create `scripts/notebook/test_diff_tracker.py`:
```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook" && "/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -m pytest test_diff_tracker.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'diff_tracker'` (the module does not exist yet).

- [ ] **Step 3: Write the implementation**

Create `scripts/notebook/diff_tracker.py`:
```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook" && "/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -m pytest test_diff_tracker.py -v
```
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override"
git add scripts/notebook/diff_tracker.py scripts/notebook/test_diff_tracker.py scripts/notebook/requirements.txt
git commit -m "feat: add notebook diff-tracker script

Reads/writes docs/notebook/.last_entry (a single commit hash) and
returns the unified diff since that commit. First run (no state file
yet) diffs only the current HEAD commit, not the whole repo history.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 2: Category reference data

**Files:**
- Create: `docs/notebook/categories.md`

**Interfaces:**
- Consumes: nothing
- Produces: a markdown reference the Task 4 skill reads when classifying a diff. Not Python — no function signature, but the exact 8 headings below are load-bearing (Task 4 references them by these exact strings).

- [ ] **Step 1: Write the reference file**

Create `docs/notebook/categories.md`:
```markdown
# Notebook Page Categories

The 8 categories, taken verbatim from the 78181A Genesis reference
notebook's own legend page (`Official 78181A Push Back Notes (2).pdf`,
page 4). Every drafted page is classified into exactly one of these.

## Strategies
Season/match strategy decisions: which goals to prioritize, offense vs.
defense split, autonomous win-point routing. Not code changes.

## Goals/Plans
Timeline, scheduling, competition goals, deciding what to work on next
and why. Not a specific technical decision - the planning around one.

## Research/Brainstorm
Investigating options before choosing one: comparing mechanism types,
watching/reading reference material, listing possible approaches. If a
decision has already been made, this is the wrong category - see
Analysis/Discussions or Build/Code instead.

## Prototypes
A specific physical or code prototype was built to test one idea -
distinct from Experiment (which measures/tests that prototype) and from
Build/Code (which is the final, kept implementation).

## Experiment
A controlled test with an independent/controlled/dependent variable,
real data collected, and a conclusion drawn from it. Reuses the
IV/CV/DV + data + conclusion template already established in this
repo's `docs/*.md` notebook pages.

## Analysis/Discussions
Comparing already-built or already-decided options with real numbers -
a benchmark result, a decision matrix, a rules/scoring breakdown. The
result already exists; this category explains what it means.

## Build/Code
The actual implementation: what got written, why, and how it works.
Default category for a plain code diff with no strategic/experimental
framing attached.

## Other
Team info, budgets, calendar, anything that doesn't fit the above 7.
```

- [ ] **Step 2: Verify it matches the reference notebook**

This has no automated test - it's a content-accuracy check against source material already extracted this session. Confirm each of the 8 headings matches page 4 of `Official 78181A Push Back Notes (2).pdf` exactly (already transcribed in this plan from that page's `pdfplumber` extraction earlier in the session): Strategies, Goals/Plans, Research/Brainstorm, Prototypes, Experiment, Analysis/Discussions, Build/Code, Other.

- [ ] **Step 3: Commit**

```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override"
git add docs/notebook/categories.md
git commit -m "docs: add notebook page category reference

Verbatim from the 78181A Genesis reference notebook's legend page,
with classification criteria for each category so the drafting skill
(Task 4) has an unambiguous rubric instead of guessing per page.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 3: Code-snippet renderer

**Files:**
- Create: `scripts/notebook/snippet_renderer.py`
- Test: `scripts/notebook/test_snippet_renderer.py`

**Interfaces:**
- Consumes: nothing new (standalone; Task 4 calls this with a file path + line range it picks from Task 1's diff output)
- Produces: `render_snippet_html(file_path: str, start_line: int, end_line: int) -> str` — returns a complete, self-contained HTML document (inline CSS, no external asset references) with the given line range syntax-highlighted. Rasterizing that HTML to a PNG happens at skill-invocation time via the Browser pane tool already used throughout this session (`mcp__Claude_Browser__computer` screenshot action against a `data:text/html,` URL or a temp file) - not a new dependency, and not unit-tested here since it requires a live browser pane. This task's automated test covers HTML generation only, which is where a real bug (wrong lines selected, broken highlighting, wrong language detection) would actually show up.

- [ ] **Step 1: Write the failing tests**

Create `scripts/notebook/test_snippet_renderer.py`:
```python
import tempfile
from pathlib import Path

from snippet_renderer import render_snippet_html


def _write_sample_cpp(tmp_path: Path) -> Path:
    content = "\n".join(
        [
            "int a = 1;",
            "int b = 2;",
            "int c = a + b;",
            "return c;",
        ]
    )
    file_path = tmp_path / "sample.cpp"
    file_path.write_text(content + "\n")
    return file_path


def test_render_snippet_html_includes_only_requested_lines(tmp_path):
    file_path = _write_sample_cpp(tmp_path)
    html = render_snippet_html(str(file_path), 2, 3)
    assert "int b = 2;" in html
    assert "int c = a + b;" in html
    assert "int a = 1;" not in html
    assert "return c;" not in html


def test_render_snippet_html_is_self_contained(tmp_path):
    file_path = _write_sample_cpp(tmp_path)
    html = render_snippet_html(str(file_path), 1, 1)
    assert "<html" in html.lower()
    assert "<style" in html.lower()
    assert "http://" not in html
    assert "https://" not in html


def test_render_snippet_html_detects_cpp_syntax(tmp_path):
    file_path = _write_sample_cpp(tmp_path)
    html = render_snippet_html(str(file_path), 1, 1)
    # Pygments emits a token class around keywords/types it recognizes;
    # "int" as a recognized type keyword is the concrete, checkable signal
    # that C++ lexing actually happened rather than plain-text fallback.
    assert 'class="kt"' in html or 'class="k"' in html
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook" && "/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -m pytest test_snippet_renderer.py -v
```
Expected: FAIL — `ModuleNotFoundError: No module named 'snippet_renderer'`.

- [ ] **Step 3: Write the implementation**

Create `scripts/notebook/snippet_renderer.py`:
```python
"""Renders a line range of a source file as self-contained, syntax-highlighted HTML."""

from pathlib import Path

from pygments import highlight
from pygments.formatters import HtmlFormatter
from pygments.lexers import guess_lexer_for_filename


def render_snippet_html(file_path: str, start_line: int, end_line: int) -> str:
    """Return a self-contained HTML document highlighting lines [start_line, end_line] (1-indexed, inclusive)."""
    lines = Path(file_path).read_text().splitlines()
    selected = lines[start_line - 1:end_line]
    code = "\n".join(selected)

    lexer = guess_lexer_for_filename(file_path, code)
    formatter = HtmlFormatter(full=True, style="monokai", noclasses=False)
    return highlight(code, lexer, formatter)
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override/scripts/notebook" && "/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -m pytest test_snippet_renderer.py -v
```
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override"
git add scripts/notebook/snippet_renderer.py scripts/notebook/test_snippet_renderer.py
git commit -m "feat: add notebook code-snippet HTML renderer

Renders a line range of a source file as self-contained, syntax-
highlighted HTML via pygments. Rasterizing to PNG happens at
skill-invocation time via the Browser pane tool, not here - this
covers the part that can actually break (line selection, language
detection, self-containment).

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 4: The `/notebook` skill

**Files:**
- Create: `.claude/skills/notebook/SKILL.md`

**Interfaces:**
- Consumes: `diff_tracker.get_last_entry`, `diff_tracker.get_diff_since`, `diff_tracker.get_current_commit`, `diff_tracker.update_last_entry` (Task 1); `docs/notebook/categories.md`'s 8 headings (Task 2); `snippet_renderer.render_snippet_html` (Task 3).
- Produces: the on-demand `/notebook` entry point. Nothing later in this plan consumes it — Phase B (a separate, not-yet-written plan) will extend this skill once the Canva plugin is authorized, but does not exist yet.

This is a playbook (instructions I follow when invoked), not code — there is no pytest step. It is verified in Step 2 by an actual dry run against this repo's real, current diff.

- [ ] **Step 1: Write the skill**

Create `.claude/skills/notebook/SKILL.md`:
```markdown
---
name: notebook
description: Draft an engineering notebook page from what changed since the last entry, in the team's established notebook voice. On-demand only - never runs automatically. Phase A (draft) only; does not place anything into Canva yet.
---

# Notebook Page Drafter

## What this does

Drafts engineering notebook page text from what actually changed in the
repo since the last time this skill ran, plus the current conversation.
Produces plain text for the user to review - does not touch Canva. (Canva
placement is a separate, not-yet-built phase - see
`docs/superpowers/specs/2026-08-17-engineering-notebook-authoring-design.md`.)

## Steps

1. Run `scripts/notebook/diff_tracker.py`'s `get_last_entry` against
   `docs/notebook/.last_entry`, then `get_diff_since` with that value
   (repo root: wherever this skill was invoked from) to get the diff to
   draft from. If this is the first-ever run, `get_last_entry` returns
   `None` and `get_diff_since` returns only the latest commit's diff, not
   the whole repo history.

2. Read `docs/notebook/categories.md`. Classify the diff into exactly one
   of its 8 headings using the criteria written there - do not invent a
   9th category or split one change across multiple pages unless the diff
   genuinely covers unrelated changes.

3. Draft the page text in the notebook's established voice: bolded
   lead-in phrases, terse technical body, a Pro/Con table if comparing
   mechanism options, the IV/CV/DV + data + conclusion structure if this
   is genuinely an Experiment-category page, a signature/date footer
   (signed by whoever is actually doing the work this session, dated with
   today's real date - never a placeholder name or date). Match the voice
   already established in this repo's `docs/*.md` notebook pages, not a
   generic engineering-report tone.

4. If the diff includes a code change worth illustrating (a genuinely
   new function, a fixed bug, a meaningfully changed algorithm - not
   every line touched), pick the specific file and line range, and call
   `scripts/notebook/snippet_renderer.py`'s `render_snippet_html` on it.
   Render that HTML to a PNG using the Browser pane tool (open the HTML,
   screenshot it) and note it as a figure in the draft.

5. Present the full draft - text and any rendered figures - to the user
   directly in chat. Stop here. Do not call `update_last_entry` yet -
   that only happens once the user has actually approved the draft AND
   (once Phase B exists) it has been placed into Canva. Approving the
   draft text alone is not the same as it being in the notebook yet.

## What this explicitly does not do

- Does not run automatically - only when the user invokes `/notebook`.
- Does not touch Canva - that is Phase B, a separate plan, blocked on
  the user authorizing the `canva` plugin.
- Does not take literal screenshots of the editor/terminal - figures are
  always rendered, syntax-highlighted code, per
  `docs/superpowers/specs/2026-08-17-engineering-notebook-authoring-design.md`.
```

- [ ] **Step 2: Dry run against this repo's real current diff**

Run:
```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override" && "/c/Users/USER/AppData/Local/Programs/Python/Python312/python.exe" -c "
import sys
sys.path.insert(0, 'scripts/notebook')
from diff_tracker import get_last_entry, get_diff_since, get_current_commit
last = get_last_entry('docs/notebook/.last_entry')
print('last entry:', last)
diff = get_diff_since('.', last)
print('diff length:', len(diff))
print(diff[:500])
"
```
Expected: `last entry: None` (no state file exists yet in this repo), a non-empty diff printed (the current `HEAD` commit's real content), no exceptions.

Then actually invoke the `/notebook` skill once by hand (as Claude, following the steps written in Step 1 against that same real diff) and confirm the output reads like the existing `docs/*.md` notebook pages in this repo - this is the actual acceptance check for this task, not the script run above, which only proves the plumbing works.

- [ ] **Step 3: Commit**

```bash
cd "C:/Users/USER/Documents/GitHub/14683A-Override"
git add .claude/skills/notebook/SKILL.md
git commit -m "feat: add /notebook skill (Phase A - draft only)

Wires diff_tracker + categories.md + snippet_renderer into an on-demand
skill that drafts notebook page text and stops - no Canva placement
yet. Verified with a dry run against this repo's real current diff.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```
