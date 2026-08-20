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
    formatter = HtmlFormatter(full=False, style="monokai", noclasses=False)
    highlighted = highlight(code, lexer, formatter)
    css = formatter.get_style_defs(".highlight")

    return (
        "<html><head><style>{css}</style></head>"
        "<body>{highlighted}</body></html>"
    ).format(css=css, highlighted=highlighted)
