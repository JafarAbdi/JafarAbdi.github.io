#!/usr/bin/env -S uv run --script
# /// script
# dependencies = ["markdown", "pygments"]
# ///
"""Build the blog: markdown in posts/ and about.md, static site out to _site/."""

import html
import pathlib
import re
import shutil
import xml.etree.ElementTree as etree

import markdown
from markdown import treeprocessors
from pygments import formatters

SITE_URL = "https://jafarabdi.github.io"
SITE_TITLE = "Jafar Uruç"
REPO_URL = "https://github.com/JafarAbdi/JafarAbdi.github.io"

ROOT = pathlib.Path(__file__).parent
OUT = ROOT / "_site"


class LinkImages(treeprocessors.Treeprocessor):
    """Link each image to its own file so clicking shows it full size."""

    def run(self, root: etree.Element) -> None:
        for parent in root.iter():
            for index, image in enumerate(parent):
                if image.tag == "img" and parent.tag != "a":
                    link = etree.Element("a", href=image.get("src", ""))
                    link.append(image)
                    parent[index] = link


def render(text: str) -> str:
    md = markdown.Markdown(
        extensions=["fenced_code", "codehilite", "tables", "toc"],
        extension_configs={"codehilite": {"guess_lang": False}},
    )
    md.treeprocessors.register(LinkImages(md), "link_images", 5)
    return md.convert(text)


def render_inline(text: str) -> str:
    rendered = render(text)
    assert rendered.startswith("<p>"), f"expected a single paragraph: {text!r}"
    assert rendered.endswith("</p>"), f"expected a single paragraph: {text!r}"
    assert rendered.count("<p>") == 1, f"expected a single paragraph: {text!r}"
    return rendered.removeprefix("<p>").removesuffix("</p>")


def parse(path: pathlib.Path) -> dict[str, str]:
    text = path.read_text(encoding="utf-8")
    assert text.startswith("---\n"), f"{path}: missing frontmatter"
    meta_block, closed, body = text.removeprefix("---\n").partition("\n---\n")
    assert closed, f"{path}: unterminated frontmatter"
    meta: dict[str, str] = {}
    for line in meta_block.splitlines():
        key, _, value = line.partition(":")
        meta[key.strip()] = value.strip()
    assert "title" in meta, f"{path}: missing title"
    assert "slug" not in meta, f"{path}: slug is derived from the filename"
    assert "body" not in meta, f"{path}: body is a reserved key"
    assert "date" not in meta, f"{path}: date comes from the posts/YYYY/MM/DD/ path"
    return meta | {"slug": path.stem, "body": body}


def write(url_path: str, title: str, content: str, template: str) -> None:
    page = template.replace("{{title}}", html.escape(title)).replace(
        "{{content}}", content
    )
    out_dir = OUT / url_path.strip("/")
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "index.html").write_text(page, encoding="utf-8")


def post_html(post: dict[str, str]) -> str:
    header = (
        f'<h1>{html.escape(post["title"])}</h1>\n<p class="date">{post["date"]}</p>\n'
    )
    footer = (
        f'\n<footer><a href="{REPO_URL}/edit/main/{post["src"]}">fix typo</a></footer>'
    )
    return header + render(post["body"]) + footer


def feed(posts: list[dict[str, str]]) -> str:
    entries = "".join(
        f"""
  <entry>
    <title>{html.escape(post["title"])}</title>
    <link href="{SITE_URL}{post["url"]}"/>
    <id>{SITE_URL}{post["url"]}</id>
    <updated>{post["date"]}T00:00:00Z</updated>
    <content type="html" xml:base="{SITE_URL}{post["url"]}">{html.escape(render(post["body"]))}</content>
  </entry>"""
        for post in posts
    )
    return f"""<?xml version="1.0" encoding="utf-8"?>
<feed xmlns="http://www.w3.org/2005/Atom">
  <title>{SITE_TITLE}</title>
  <link href="{SITE_URL}/"/>
  <link rel="self" href="{SITE_URL}/atom.xml"/>
  <id>{SITE_URL}/</id>
  <updated>{posts[0]["date"]}T00:00:00Z</updated>
  <author><name>{SITE_TITLE}</name></author>{entries}
</feed>
"""


def main() -> None:
    template = (ROOT / "template.html").read_text(encoding="utf-8")
    shutil.rmtree(OUT, ignore_errors=True)
    OUT.mkdir()
    highlight_light = formatters.HtmlFormatter(style="default")
    highlight_dark = formatters.HtmlFormatter(style="native")
    (OUT / "style.css").write_text(
        highlight_light.get_style_defs(".codehilite")
        + "\n@media (prefers-color-scheme: dark) {\n"
        + highlight_dark.get_style_defs(".codehilite")
        + "\n}\n"
        + (ROOT / "style.css").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    shutil.copytree(ROOT / "static", OUT / "static")

    posts = []
    for path in sorted((ROOT / "posts").glob("*/*/*/*.md")):
        date = "-".join(path.relative_to(ROOT / "posts").parts[:3])
        assert re.fullmatch(r"\d{4}-\d{2}-\d{2}", date), (
            f"{path}: expected posts/YYYY/MM/DD/<slug>.md"
        )
        posts.append(
            parse(path)
            | {
                "date": date,
                "src": str(path.relative_to(ROOT)),
                "url": f"/{date.replace('-', '/')}/{path.stem}/",
            }
        )
    assert posts, "no posts found"
    posts.sort(key=lambda post: post["date"], reverse=True)

    for post in posts:
        shutil.copytree(
            (ROOT / post["src"]).parent,
            OUT / post["url"].strip("/"),
            ignore=shutil.ignore_patterns("*.md"),
        )
        write(post["url"], post["title"], post_html(post), template)

    for path in ROOT.glob("*.md"):
        if path.name in ("README.md", "news.md"):
            continue
        page = parse(path)
        write(
            f"/{page['slug']}/",
            page["title"],
            f"<h1>{html.escape(page['title'])}</h1>\n" + render(page["body"]),
            template,
        )

    entries = [
        (post["date"], f'<a href="{post["url"]}">{html.escape(post["title"])}</a>')
        for post in posts
    ]
    news = parse(ROOT / "news.md")
    for line in news["body"].strip().splitlines():
        assert line.startswith("- "), f"news.md: expected '- YYYY-MM-DD text': {line}"
        date, _, text = line.removeprefix("- ").partition(" ")
        assert re.fullmatch(r"\d{4}-\d{2}-\d{2}", date), (
            f"news.md: expected '- YYYY-MM-DD text': {line}"
        )
        entries.append((date, f"<span>{render_inline(text)}</span>"))
    entries.sort(reverse=True)
    items = "\n".join(
        f"<li><time>{date}</time>{content}</li>" for date, content in entries
    )
    links = """<footer>
<a href="https://github.com/JafarAbdi">GitHub</a>
<a href="https://www.linkedin.com/in/jafar-uruc">LinkedIn</a>
<a href="https://bsky.app/profile/jafar-uruc.bsky.social">Bluesky</a>
<a href="https://x.com/JafarUruc">Twitter</a>
<a href="https://scholar.google.com/citations?user=Nx-CYEMAAAAJ">Google Scholar</a>
</footer>"""
    write("/", SITE_TITLE, f'<ul class="posts">\n{items}\n</ul>\n{links}', template)

    (OUT / "atom.xml").write_text(feed(posts), encoding="utf-8")
    print(f"built {len(posts)} post(s) -> {OUT}")


if __name__ == "__main__":
    main()
