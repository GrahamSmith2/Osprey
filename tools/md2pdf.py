"""
md2pdf.py — render a project Markdown doc to a print-ready PDF.

    python tools/md2pdf.py docs/LEADS_BRIEF.md [out.pdf]

Why this exists rather than pandoc: none of pandoc, weasyprint or python-markdown
are installed on the build machine, and adding a LaTeX toolchain to print a
two-page agenda is not proportionate. Microsoft Edge ships with Windows and will
print HTML to PDF headlessly with full CSS and table support, which is all this
needs.

The Markdown subset handled is the one these docs actually use: ATX headings,
pipe tables, bullet and numbered lists, blockquotes, horizontal rules, inline
bold/italic/code, and links. It is deliberately not a general Markdown engine.
"""
import html
import os
import re
import subprocess
import sys
import tempfile

EDGE_CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]

CSS = """
@page { size: A4; margin: 17mm 16mm 20mm 16mm; }
* { box-sizing: border-box; }
body {
  font: 10.2pt/1.5 Georgia, "Times New Roman", serif;
  color: #14181d; margin: 0;
  -webkit-print-color-adjust: exact; print-color-adjust: exact;
}
h1, h2, h3, th, .meta { font-family: "Segoe UI", Arial, sans-serif; }
h1 { font-size: 21pt; line-height: 1.15; margin: 0 0 4pt; letter-spacing: -.01em; }
h2 {
  font-size: 13pt; margin: 20pt 0 7pt; padding-bottom: 4pt;
  border-bottom: 1.5pt solid #14181d; break-after: avoid;
}
h3 { font-size: 10.8pt; margin: 14pt 0 5pt; break-after: avoid; }
h1 + p, h2 + p, h3 + p, h2 + table, h3 + table { break-before: avoid; }
p { margin: 0 0 7pt; }
ul, ol { margin: 0 0 8pt; padding-left: 16pt; }
li { margin-bottom: 3pt; }
hr { border: 0; border-top: .6pt solid #c3cbd3; margin: 15pt 0; }
a { color: #14181d; text-decoration: none; border-bottom: .5pt solid #98a4b0; }
code {
  font: .88em ui-monospace, "Cascadia Mono", Consolas, monospace;
  background: #eef1f4; padding: .5pt 3pt; border-radius: 2pt;
}
strong { font-weight: 700; }
blockquote {
  margin: 9pt 0; padding: 7pt 11pt; background: #f2f5f8;
  border-left: 2.5pt solid #14181d;
}
blockquote p:last-child { margin-bottom: 0; }
table {
  width: 100%; border-collapse: collapse; margin: 8pt 0 11pt;
  font-size: 9.2pt; break-inside: auto;
}
th {
  text-align: left; font-size: 8pt; text-transform: uppercase;
  letter-spacing: .06em; padding: 5pt 7pt 4pt;
  border-bottom: 1.2pt solid #14181d; vertical-align: bottom;
}
td { padding: 5pt 7pt; border-bottom: .5pt solid #d5dce2; vertical-align: top; }
tr { break-inside: avoid; }
tbody tr:nth-child(even) td { background: #f6f8fa; }
td:first-child, th:first-child { padding-left: 0; }
td:last-child, th:last-child { padding-right: 0; }
.meta { font-size: 8.5pt; color: #5b6773; margin: 0 0 13pt; }
figure { margin: 10pt 0 12pt; break-inside: avoid; text-align: center; }
figure img { max-width: 100%; max-height: 95mm; height: auto; border: .5pt solid #d5dce2; }
figcaption {
  font-family: "Segoe UI", Arial, sans-serif; font-size: 8.3pt; color: #5b6773;
  margin-top: 4pt; text-align: left;
}
p > figure, p figure { display: block; }
"""

INLINE = [
    # images first, so the link rule below does not eat the ![...](...) form.
    # A caption ending in "|right" floats the figure to the right (compact mode).
    (re.compile(r"!\[([^\]|]*)(?:\|(right))?\]\(([^)]+)\)"),
        lambda m: f'<figure class="{m.group(2) or ""}"><img src="{m.group(3)}" alt="{m.group(1)}">'
                  f'<figcaption>{m.group(1)}</figcaption></figure>'),
    (re.compile(r"`([^`]+)`"),                lambda m: f"<code>{html.escape(m.group(1))}</code>"),
    (re.compile(r"\*\*([^*]+)\*\*"),          lambda m: f"<strong>{m.group(1)}</strong>"),
    (re.compile(r"(?<![*\w])\*([^*\n]+)\*"),  lambda m: f"<em>{m.group(1)}</em>"),
    (re.compile(r"\[([^\]]+)\]\(([^)]+)\)"),  lambda m: f'<a href="{m.group(2)}">{m.group(1)}</a>'),
]


def inline(text):
    """Escape, then apply inline markup. Code spans escape their own content."""
    out = html.escape(text, quote=False)
    for pattern, repl in INLINE:
        out = pattern.sub(repl, out)
    return out


def split_row(line):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def convert(md):
    lines = md.split("\n")
    out, i, n = [], 0, len(lines)
    list_open = None

    def close_list():
        nonlocal list_open
        if list_open:
            out.append(f"</{list_open}>")
            list_open = None

    while i < n:
        line = lines[i]
        stripped = line.strip()

        # table: a header row followed by a |---|---| separator
        if (stripped.startswith("|") and i + 1 < n
                and re.match(r"^\|[\s:|-]+\|$", lines[i + 1].strip())):
            close_list()
            head = split_row(stripped)
            out.append("<table><thead><tr>"
                       + "".join(f"<th>{inline(c)}</th>" for c in head)
                       + "</tr></thead><tbody>")
            i += 2
            while i < n and lines[i].strip().startswith("|"):
                cells = split_row(lines[i].strip())
                cells += [""] * (len(head) - len(cells))
                out.append("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in cells[:len(head)]) + "</tr>")
                i += 1
            out.append("</tbody></table>")
            continue

        if not stripped:
            close_list()
            i += 1
            continue

        if stripped.startswith("---") and set(stripped) <= set("-"):
            close_list()
            out.append("<hr>")
            i += 1
            continue

        m = re.match(r"^(#{1,6})\s+(.*)$", stripped)
        if m:
            close_list()
            lvl = len(m.group(1))
            out.append(f"<h{lvl}>{inline(m.group(2))}</h{lvl}>")
            i += 1
            continue

        if stripped.startswith(">"):
            close_list()
            block = []
            while i < n and lines[i].strip().startswith(">"):
                block.append(lines[i].strip().lstrip(">").strip())
                i += 1
            body = " ".join(b for b in block if b)
            out.append(f"<blockquote><p>{inline(body)}</p></blockquote>")
            continue

        m = re.match(r"^[-*]\s+(.*)$", stripped)
        if m:
            if list_open != "ul":
                close_list()
                out.append("<ul>")
                list_open = "ul"
            out.append(f"<li>{inline(m.group(1))}</li>")
            i += 1
            continue

        m = re.match(r"^\d+\.\s+(.*)$", stripped)
        if m:
            if list_open != "ol":
                close_list()
                out.append("<ol>")
                list_open = "ol"
            out.append(f"<li>{inline(m.group(1))}</li>")
            i += 1
            continue

        close_list()
        para = [stripped]
        i += 1
        while i < n and lines[i].strip() and not re.match(
                r"^(#{1,6}\s|[-*]\s|\d+\.\s|>|\||---)", lines[i].strip()):
            para.append(lines[i].strip())
            i += 1
        out.append(f"<p>{inline(' '.join(para))}</p>")

    close_list()
    return "\n".join(out)


COMPACT_CSS = """
@page { margin: 12mm 13mm 12mm 13mm; }
body { font-size: 9pt; line-height: 1.38; }
h1 { font-size: 16pt; margin-bottom: 2pt; }
h2 { font-size: 10.5pt; margin: 9pt 0 4pt; padding-bottom: 2pt; }
p { margin: 0 0 4pt; }
ul, ol { margin: 0 0 4pt; padding-left: 13pt; } li { margin-bottom: 1pt; }
table { font-size: 8.3pt; margin: 4pt 0 7pt; }
th { font-size: 7.2pt; padding: 3pt 5pt 2pt; } td { padding: 2.5pt 5pt; }
blockquote { margin: 5pt 0; padding: 4pt 8pt; }
figure { margin: 4pt 0 6pt; }
figure.right { float: right; width: 34%; margin: 0 0 4pt 10pt; }
figure.right img { max-height: 44mm; width: auto; }
h2 { clear: both; }
figcaption { font-size: 7.4pt; margin-top: 2pt; }
hr { margin: 7pt 0; }
"""


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    compact = "--compact" in sys.argv
    src = os.path.abspath(args[0])
    dst = os.path.abspath(args[1]) if len(args) > 1 else os.path.splitext(src)[0] + ".pdf"

    edge = next((p for p in EDGE_CANDIDATES if os.path.exists(p)), None)
    if not edge:
        sys.exit("No Edge or Chrome found; cannot render PDF.")

    md = open(src, encoding="utf-8").read()
    title = html.escape(os.path.basename(src))
    m = re.search(r"^#\s+(.*)$", md, re.M)
    if m:
        title = html.escape(re.sub(r"[*`]", "", m.group(1)))

    body = convert(md)

    # Image paths in the Markdown are relative to the .md file; Edge is handed
    # a temp HTML file somewhere else, so resolve them to absolute file:// URLs.
    base = os.path.dirname(src)
    def _abs(m):
        p = m.group(1)
        if re.match(r"^(https?:|file:|data:)", p):
            return m.group(0)
        full = os.path.normpath(os.path.join(base, p)).replace("\\", "/")
        return f'src="file:///{full}"'
    body = re.sub(r'src="([^"]+)"', _abs, body)

    css = CSS + (COMPACT_CSS if compact else "")
    page = (f"<!doctype html><html><head><meta charset='utf-8'>"
            f"<title>{title}</title><style>{css}</style></head>"
            f"<body>{body}</body></html>")

    tmp = os.path.join(tempfile.gettempdir(), "md2pdf_tmp.html")
    open(tmp, "w", encoding="utf-8").write(page)

    subprocess.run(
        [edge, "--headless", "--disable-gpu", "--no-pdf-header-footer",
         f"--print-to-pdf={dst}", "file:///" + tmp.replace("\\", "/")],
        check=True, capture_output=True, timeout=120,
    )
    print(f"{os.path.basename(src)} -> {dst}  ({os.path.getsize(dst)/1024:.0f} KB)")


if __name__ == "__main__":
    main()
