# rebuild _site/ (uv reads build.py's inline metadata and provides `markdown`)
build:
    uv run build.py

# build, serve at http://localhost:<port>, and rebuild on change
serve port="8080":
    uv run serve.py {{port}}

# lint and format python with default ruff
lint:
    uvx ruff check --fix .
    uvx ruff format .

# scaffold posts/YYYY/MM/DD/<slug>.md under today's date
new slug:
    mkdir -p posts/$(date +%Y/%m/%d)
    test ! -f posts/$(date +%Y/%m/%d)/{{slug}}.md
    printf -- '---\ntitle: TODO\n---\n\n' > posts/$(date +%Y/%m/%d)/{{slug}}.md
    echo "created posts/$(date +%Y/%m/%d)/{{slug}}.md"
