# Publishing Documentation to the Web

This guide explains how to build and publish the JaguarEngine documentation website.

## Overview

JaguarEngine documentation can be published using several static site generators:

| Generator | Best For | Complexity |
|-----------|----------|------------|
| **MkDocs + Material** | Simple, beautiful docs | Easy |
| **Docusaurus** | React-based, feature-rich | Medium |
| **VitePress** | Vue-based, fast | Medium |
| **Doxygen** | API reference from code | Medium |

This guide focuses on **MkDocs with Material theme**, the recommended approach.

---

## Method 1: MkDocs (Recommended)

### Prerequisites

```bash
# Install MkDocs and Material theme
pip install mkdocs mkdocs-material

# Optional: Install useful plugins
pip install mkdocs-minify-plugin
pip install mkdocs-git-revision-date-localized-plugin
pip install mkdocs-awesome-pages-plugin
```

### Configuration

Create `mkdocs.yml` in the project root:

```yaml
# Project information
site_name: JaguarEngine Documentation
site_url: https://jaguarcode.github.io/JaguarEngine/
site_author: JaguarEngine Contributors
site_description: >-
  Next-generation multi-domain physics simulation platform
  for defense modeling and simulation applications.

# Repository
repo_name: jaguarcode/JaguarEngine
repo_url: https://github.com/jaguarcode/JaguarEngine
edit_uri: edit/main/docs/web/

# Copyright
copyright: Copyright &copy; 2025 JaguarEngine Contributors

# Configuration
theme:
  name: material
  custom_dir: docs/overrides
  features:
    - navigation.tabs
    - navigation.tabs.sticky
    - navigation.sections
    - navigation.expand
    - navigation.path
    - navigation.top
    - navigation.footer
    - search.suggest
    - search.highlight
    - search.share
    - content.tabs.link
    - content.code.copy
    - content.code.annotate
  palette:
    - scheme: default
      primary: indigo
      accent: indigo
      toggle:
        icon: material/brightness-7
        name: Switch to dark mode
    - scheme: slate
      primary: indigo
      accent: indigo
      toggle:
        icon: material/brightness-4
        name: Switch to light mode
  font:
    text: Roboto
    code: Roboto Mono
  icon:
    repo: fontawesome/brands/github
  logo: assets/logo.svg
  favicon: assets/favicon.png

# Plugins
plugins:
  - search:
      separator: '[\s\-,:!=\[\]()"`/]+|\.(?!\d)|&[lg]t;|(?!\b)(?=[A-Z][a-z])'
  - minify:
      minify_html: true

# Extensions
markdown_extensions:
  - abbr
  - admonition
  - attr_list
  - def_list
  - footnotes
  - md_in_html
  - toc:
      permalink: true
  - pymdownx.arithmatex:
      generic: true
  - pymdownx.betterem:
      smart_enable: all
  - pymdownx.caret
  - pymdownx.details
  - pymdownx.emoji:
      emoji_index: !!python/name:material.extensions.emoji.twemoji
      emoji_generator: !!python/name:material.extensions.emoji.to_svg
  - pymdownx.highlight:
      anchor_linenums: true
      line_spans: __span
      pygments_lang_class: true
  - pymdownx.inlinehilite
  - pymdownx.keys
  - pymdownx.magiclink:
      repo_url_shorthand: true
      user: jaguarcode
      repo: JaguarEngine
  - pymdownx.mark
  - pymdownx.smartsymbols
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:pymdownx.superfences.fence_code_format
  - pymdownx.tabbed:
      alternate_style: true
  - pymdownx.tasklist:
      custom_checkbox: true
  - pymdownx.tilde

# Page tree
nav:
  - Home: index.md
  - Getting Started:
    - Installation: getting-started/installation.md
    - Quick Start: getting-started/quickstart.md
    - First Simulation: getting-started/first-simulation.md
  - Core Concepts:
    - Overview: concepts/overview.md
    - Entities: concepts/entities.md
    - Coordinate Systems: concepts/coordinates.md
    - Force Generators: concepts/force-generators.md
  - Domains:
    - Air: domains/air.md
    - Land: domains/land.md
    - Sea: domains/sea.md
    - Space: domains/space.md
  - API Reference:
    - Overview: api/overview.md
    - Core: api/core.md
    - Physics: api/physics.md
    - Configuration: api/configuration.md
    - Python: api/python.md
    - Lua: api/lua.md
    - GPU Compute: api/gpu.md
    - Cloud Burst: api/cloud.md
    - Digital Thread: api/thread.md
    - Machine Learning: api/ml.md
    - Sensors: api/sensors.md
    - Federation: api/federation.md
    - XR Integration: api/xr.md
  - Tutorials:
    - Examples: tutorials/examples.md
    - Multi-Entity: tutorials/multi-entity.md
    - Terrain: tutorials/terrain.md
  - Advanced:
    - Architecture: advanced/architecture.md
    - Custom Models: advanced/custom-models.md
    - Networking: advanced/networking.md
  - Contributing: contributing.md

# Extra
extra:
  version:
    provider: mike
  social:
    - icon: fontawesome/brands/github
      link: https://github.com/jaguarcode/JaguarEngine
    - icon: fontawesome/brands/twitter
      link: https://twitter.com/jaguarengine
  analytics:
    provider: google
    property: G-XXXXXXXXXX

extra_css:
  - stylesheets/extra.css

extra_javascript:
  - javascripts/mathjax.js
  - https://polyfill.io/v3/polyfill.min.js?features=es6
  - https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js
```

### Build and Preview

```bash
# Preview locally (live reload)
mkdocs serve

# Build static site
mkdocs build

# Output is in site/ directory
ls site/
```

### Directory Structure

```
JaguarEngine/
├── mkdocs.yml              # MkDocs configuration
├── docs/
│   ├── web/                # Documentation source
│   │   ├── index.md
│   │   ├── getting-started/
│   │   ├── concepts/
│   │   ├── domains/
│   │   ├── api/
│   │   ├── tutorials/
│   │   └── advanced/
│   ├── overrides/          # Theme customization
│   │   └── main.html
│   └── assets/             # Images, logos
│       ├── logo.svg
│       └── favicon.png
└── site/                   # Built output (gitignored)
```

---

## Method 2: GitHub Pages Deployment

### Automatic Deployment with GitHub Actions

Create `.github/workflows/docs.yml`:

```yaml
name: Documentation

on:
  push:
    branches:
      - main
    paths:
      - 'docs/**'
      - 'mkdocs.yml'
  pull_request:
    branches:
      - main
    paths:
      - 'docs/**'
      - 'mkdocs.yml'

permissions:
  contents: write

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - name: Configure Git Credentials
        run: |
          git config user.name github-actions[bot]
          git config user.email 41898282+github-actions[bot]@users.noreply.github.com

      - uses: actions/setup-python@v5
        with:
          python-version: 3.x

      - run: echo "cache_id=$(date --utc '+%V')" >> $GITHUB_ENV

      - uses: actions/cache@v4
        with:
          key: mkdocs-material-${{ env.cache_id }}
          path: .cache
          restore-keys: |
            mkdocs-material-

      - name: Install dependencies
        run: |
          pip install mkdocs-material
          pip install mkdocs-minify-plugin

      - name: Deploy to GitHub Pages
        if: github.event_name != 'pull_request'
        run: mkdocs gh-deploy --force

      - name: Build (PR preview)
        if: github.event_name == 'pull_request'
        run: mkdocs build
```

### Enable GitHub Pages

1. Go to repository **Settings** > **Pages**
2. Set **Source** to "Deploy from a branch"
3. Set **Branch** to `gh-pages` / `/ (root)`
4. Save

Your documentation will be available at:
`https://jaguarcode.github.io/JaguarEngine/`

---

## Method 3: Vercel Deployment

### Setup

1. Connect your GitHub repository to Vercel
2. Configure build settings:

```json
{
  "buildCommand": "pip install mkdocs-material && mkdocs build",
  "outputDirectory": "site",
  "installCommand": "pip install mkdocs mkdocs-material"
}
```

### vercel.json

```json
{
  "version": 2,
  "builds": [
    {
      "src": "mkdocs.yml",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "site"
      }
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "/$1"
    }
  ]
}
```

---

## Method 4: Netlify Deployment

### netlify.toml

```toml
[build]
  command = "pip install mkdocs-material mkdocs-minify-plugin && mkdocs build"
  publish = "site"

[build.environment]
  PYTHON_VERSION = "3.11"

[[redirects]]
  from = "/*"
  to = "/index.html"
  status = 200
```

---

## Doxygen Integration

Generate API documentation from code comments:

### docs/Doxyfile

```
PROJECT_NAME           = "JaguarEngine"
PROJECT_NUMBER         = 0.5.0
PROJECT_BRIEF          = "Multi-Domain Physics Simulation"
OUTPUT_DIRECTORY       = docs/api-reference
INPUT                  = include/jaguar
RECURSIVE              = YES
EXTRACT_ALL            = YES
GENERATE_HTML          = YES
GENERATE_LATEX         = NO
HTML_OUTPUT            = html
USE_MDFILE_AS_MAINPAGE = docs/pages/mainpage.md
```

### Generate and Integrate

```bash
# Generate Doxygen output
doxygen docs/Doxyfile

# Link from MkDocs
# Add to mkdocs.yml nav:
# - API Reference: api-reference/html/index.html
```

---

## Custom Styling

### docs/stylesheets/extra.css

```css
/* Custom primary color */
:root {
  --md-primary-fg-color: #1a237e;
  --md-primary-fg-color--light: #3949ab;
  --md-primary-fg-color--dark: #0d1642;
}

/* Hero section styling */
.hero {
  text-align: center;
  padding: 2rem;
  background: linear-gradient(135deg, #1a237e 0%, #3949ab 100%);
  color: white;
  border-radius: 8px;
  margin-bottom: 2rem;
}

.hero h1 {
  font-size: 3rem;
  margin-bottom: 0.5rem;
}

.tagline {
  font-size: 1.5rem;
  opacity: 0.9;
}

/* Feature grid */
.features {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1.5rem;
  margin: 2rem 0;
}

.features h3 {
  color: var(--md-primary-fg-color);
}

/* Code blocks */
.highlight pre {
  border-radius: 8px;
}

/* Tables */
.md-typeset table:not([class]) {
  border-collapse: collapse;
  border-radius: 8px;
  overflow: hidden;
}
```

---

## Version Management

Use `mike` for versioned documentation:

```bash
# Install mike
pip install mike

# Deploy version
mike deploy --push --update-aliases 0.5.0 latest

# Set default version
mike set-default --push latest

# List versions
mike list
```

Update `mkdocs.yml`:

```yaml
extra:
  version:
    provider: mike
```

---

## Checklist for Publishing

- [ ] Create `mkdocs.yml` configuration
- [ ] Organize documentation in `docs/web/`
- [ ] Add logo and favicon to `docs/assets/`
- [ ] Create custom CSS in `docs/stylesheets/extra.css`
- [ ] Set up GitHub Actions for automatic deployment
- [ ] Enable GitHub Pages in repository settings
- [ ] Configure custom domain (optional)
- [ ] Add Google Analytics (optional)
- [ ] Set up versioning with `mike` (optional)
- [ ] Test build locally with `mkdocs serve`
- [ ] Review all pages for broken links
- [ ] Verify code examples are correct
- [ ] Add search functionality
- [ ] Test on mobile devices

---

## Quick Start Commands

```bash
# Install tools
pip install mkdocs mkdocs-material

# Preview locally
mkdocs serve

# Build
mkdocs build

# Deploy to GitHub Pages
mkdocs gh-deploy

# Deploy versioned docs
mike deploy 0.5.0 latest --push
```

---

## Troubleshooting

### Build Fails

```bash
# Check for syntax errors
mkdocs build --strict

# Verbose output
mkdocs build -v
```

### Missing Pages

Ensure all pages in `nav:` section of `mkdocs.yml` exist in `docs/web/`.

### Broken Links

Use the `mkdocs-redirects` plugin:

```yaml
plugins:
  - redirects:
      redirect_maps:
        'old-page.md': 'new-page.md'
```

### GitHub Pages Not Updating

1. Check GitHub Actions workflow status
2. Verify `gh-pages` branch exists
3. Clear browser cache
4. Wait a few minutes for propagation

---

## Resources

- [MkDocs Documentation](https://www.mkdocs.org/)
- [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
- [GitHub Pages](https://pages.github.com/)
- [Doxygen Manual](https://www.doxygen.nl/manual/)
