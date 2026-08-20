import re
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


def _strip_tags(html: str) -> str:
    """Pygments wraps every token (including whitespace) in its own <span>,
    so raw source text never appears as a literal contiguous substring in
    the markup. Strip tags first to check the same intent - that only the
    requested lines' content is present - against the rendered text."""
    return re.sub(r"<[^>]+>", "", html)


def test_render_snippet_html_includes_only_requested_lines(tmp_path):
    file_path = _write_sample_cpp(tmp_path)
    html = render_snippet_html(str(file_path), 2, 3)
    text = _strip_tags(html)
    assert "int b = 2;" in text
    assert "int c = a + b;" in text
    assert "int a = 1;" not in text
    assert "return c;" not in text


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
    assert 'class="kt"' in html or 'class="k"' in html
