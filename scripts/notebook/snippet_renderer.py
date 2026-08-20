"""Renders a line range of a source file as self-contained, syntax-highlighted HTML."""

from pathlib import Path

from pygments import highlight
from pygments.formatters import HtmlFormatter
from pygments.lexers import TextLexer, guess_lexer_for_filename
from pygments.util import ClassNotFound


def render_snippet_html(file_path: str, start_line: int, end_line: int) -> str:
    """Return a self-contained HTML document highlighting lines [start_line, end_line] (1-indexed, inclusive)."""
    lines = Path(file_path).read_text().splitlines()

    if start_line < 1:
        raise ValueError(f"start_line must be >= 1, got {start_line}")
    if start_line > end_line:
        raise ValueError(f"start_line ({start_line}) must be <= end_line ({end_line})")
    if start_line > len(lines):
        raise ValueError(
            f"start_line ({start_line}) is past the end of {file_path} ({len(lines)} lines)"
        )

    selected = lines[start_line - 1:end_line]
    code = "\n".join(selected)

    try:
        lexer = guess_lexer_for_filename(file_path, code)
    except ClassNotFound:
        lexer = TextLexer()
    formatter = HtmlFormatter(full=False, style="monokai", noclasses=False)
    highlighted = highlight(code, lexer, formatter)
    css = formatter.get_style_defs(".highlight")

    return (
        "<html><head><style>{css}</style></head>"
        "<body>{highlighted}</body></html>"
    ).format(css=css, highlighted=highlighted)
