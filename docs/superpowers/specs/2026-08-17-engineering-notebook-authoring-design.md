# Engineering Notebook Authoring System — Design

**Status:** Approved by user (2026-08-17), ready for implementation planning.

## Goal

An on-demand system that drafts and places pages into the team's real, existing
Canva engineering notebook while the user works on code — reusing the same
notebook voice/structure already established in this repo
(`docs/path_following_notebook.md`, `docs/pid_tuning_notebook.md`,
`~/.claude/.../memory/engineering-notebook-style.md`), but rendered into the
actual visual Canva format the team submits, not markdown.

## Reference material

- `Official 78181A Push Back Notes (2).pdf` (191 pages, team 78181A Genesis,
  2025-2026 season) — the structural reference for this design: cover →
  executive summary → table of contents → an 8-category color-coding legend
  page → chapter divider pages → content pages tagged by category, with
  figures, Pro/Con callouts, and rulebook citations → per-skills-run divider
  pages at the end.
- The 8 page categories, taken directly from that notebook's own legend page,
  are the categorization scheme this system classifies every drafted page
  into: **Strategies, Goals/Plans, Research/Brainstorm, Prototypes,
  Experiment, Analysis/Discussions, Build/Code, Other.**
- Existing notebook-voice conventions already captured in this session's
  memory (bolded lead-ins, terse technical body, Pro/Con and IV/CV/DV
  templates for experiment pages, signature/date footers) carry over
  unchanged — this system is a new *delivery mechanism* for that voice, not a
  new voice.

## Non-goals (explicitly out of scope)

- **No automatic/passive triggering.** The user runs the system on demand
  (e.g. a `/notebook` command); it never generates a page without being asked.
- **No building a Canva design system from scratch.** The user already has a
  template with the notebook's visual style in place; this system duplicates
  and fills pages within it, it does not design pages.
- **No external AI image generation.** No such tool is available in this
  environment (checked: no image-generation tool, no Canva plugin visible
  until mid-session). Decorative images are sourced from Canva's own
  built-in stock/element library as part of the Canva placement step, not
  generated separately.
- **No support for exporting into a *new* Canva project or template.** The
  system operates on the user's one existing engineering-notebook project.

## Architecture — two phases

Canva editing (even via the real plugin, not browser automation) is real API
work with real failure modes, and running it against content the user hasn't
seen yet risks wasted API calls on a draft they'd want changed. The system is
therefore split into a fast, cheap, fully-local phase and a slower phase that
only runs after explicit approval.

```
git diff (since last entry) + this session's conversation
  │
  ▼
PHASE A — Draft (fast, text-only, no external dependency)
  1. classify the change against the 8 notebook categories
  2. draft the page text in the established notebook voice
  3. show the draft to the user in chat
  │
  ▼  [user approves or edits]
  │
  ▼
PHASE B — Place (Canva, requires the canva plugin authorized)
  1. render a code-snippet image, if the diff warrants one
  2. duplicate the matching template page in the user's Canva project
  3. canva:edit-design — set title/body text, insert the code-snippet image,
     place a matching decorative element from Canva's own library
  4. canva:get-design-feedback — verify the result
  │
  ▼
update the diff-tracker state file to the current commit
```

Phase A has no dependency on Canva at all and is usable immediately. Phase B
requires the user to have authorized the `canva` plugin (via their claude.ai
connector settings) — this is a real, external dependency this design cannot
satisfy on its own, and Phase B is inert until it's done.

## Components

### 1. Diff tracker

A small state file, `docs/notebook/.last_entry`, recording the git commit
hash of the last successful notebook run. Each invocation reads the diff
between that commit and `HEAD` instead of the whole repository history.
Updated only after Phase B completes successfully — if Phase B fails or is
skipped, the next run still picks up the same range of changes rather than
silently skipping content that never made it into the notebook.

### 2. Content drafter

Consumes the diff range plus the current conversation. Responsibilities:

- Classify the change against the 8 categories (a code fix → Build/Code; a
  controller comparison with real numbers → Analysis/Discussions; a new
  mechanism idea → Research/Brainstorm; a season-strategy decision →
  Strategies; etc.)
- Draft the page text in the notebook's established voice — bolded
  lead-in phrases, terse technical body, IV/CV/DV structure for anything
  that is actually an experiment, Pro/Con tables for mechanism comparisons,
  a signature/date footer.
- Present the draft in chat, plain text, for approval before Phase B ever
  runs.

### 3. Code-snippet renderer

Takes the relevant diff hunk or function (as selected by the content
drafter, not the whole diff indiscriminately) and renders it as a clean,
syntax-highlighted image — not a literal screen capture — matching the
"rendered code snippets, not screenshots" decision from brainstorming. This
becomes a figure for Phase B to place.

### 4. Canva placement driver

Wraps the actual `canva:edit-design` and `canva:get-design-feedback` plugin
calls: duplicate the correct template page, set its text/image content,
verify the result. Isolated into its own component specifically so Phase A
(content drafter, code-snippet renderer) can be built, tested, and used
*before* the Canva plugin is authorized — the placement driver is the only
piece that depends on it.

## Open item to resolve during implementation, not architecture

**Category → Canva template page mapping.** The system needs to know, for
each of the 8 categories, which existing page/design in the user's Canva
project to duplicate as the starting point. This cannot be determined until
the Canva plugin is authorized and the actual project structure is visible —
it does not change any component boundary above, so it is deferred to the
implementation plan rather than blocking this spec.

## Error handling

- **Phase A never fails in a way that loses content** — it's local text
  generation; worst case the draft needs another pass, nothing was
  committed anywhere external yet.
- **Phase B failures are real API errors, not silent UI-automation
  mislicks.** If `canva:edit-design` reports a failure partway through a
  page (e.g. the target design ID is wrong, a field name doesn't match),
  stop immediately, report exactly which step and design failed, and leave
  the already-approved Phase A draft text intact and available — never
  advance the diff-tracker state file past a failed run, and never leave a
  half-filled Canva page indistinguishable from a finished one.
- If the Canva plugin is not yet authorized, Phase B refuses to start with
  a clear message rather than attempting calls that will fail — Phase A
  still runs and produces a usable draft either way.

## Testing / verification

There is no meaningful unit-test surface for "did the right text land on the
right Canva page" — verification is:

- **Phase A:** the draft is held to the same voice/structure scrutiny
  already established this session for `docs/*.md` notebook pages, before
  it's shown to the user for approval.
- **Phase B:** `canva:get-design-feedback` after every placement, plus the
  design is a real, inspectable Canva page the user can open directly —
  not a claim taken on faith from a tool's reported success.

## Dependencies

- **`canva` plugin, authorized by the user.** Blocking for Phase B only.
  Cannot be completed from a non-interactive session — the user authorizes
  it via claude.ai connector settings.
- No new dependencies for Phase A — it uses only git and the existing
  conversation/session context.
