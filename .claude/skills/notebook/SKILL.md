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

   ```
   "C:\Users\USER\AppData\Local\Programs\Python\Python312\python.exe" -c "
   import sys
   sys.path.insert(0, 'scripts/notebook')
   from diff_tracker import get_last_entry, get_diff_since, get_current_commit
   last = get_last_entry('docs/notebook/.last_entry')
   print('last entry:', last)
   diff = get_diff_since('.', last)
   print(diff)
   "
   ```

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

   ```
   "C:\Users\USER\AppData\Local\Programs\Python\Python312\python.exe" -c "
   import sys
   sys.path.insert(0, 'scripts/notebook')
   from snippet_renderer import render_snippet_html
   html = render_snippet_html('path/to/file.ext', START_LINE, END_LINE)
   from pathlib import Path
   out = Path('docs/notebook/.snippet_preview.html')
   out.write_text(html, encoding='utf-8')
   print('wrote', out.resolve())
   "
   ```

   Render that HTML to a PNG using the Browser pane tool (open the HTML
   file just written, screenshot it) and note it as a figure in the
   draft.

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
