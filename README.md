# jafarabdi.github.io

Suckless blog: markdown in, HTML out.

- `posts/YYYY/MM/DD/slug.md`: a post. Date from the path, URL mirrors it, images sit next to the post. Frontmatter has `title:` and optional comma-separated `tags:`; tags are lowercase URL-safe names.
- `about.md`: a page. Any root `*.md` becomes `/name/` (except this README).
- `news.md`: `- YYYY-MM-DD text` bullets, merged into the homepage list.
- `build.py`: the generator. `serve.py`: dev server. `template.html` + `style.css`: the design. `static/`: site assets.

## Use

```bash
just new my-post   # scaffold today's post
just serve         # http://localhost:8080, rebuilds on change
```

Push to main and GitHub Actions deploys.
