# Contributing to MuJoCo Toolbox

Thank you for your interest in contributing!  
This guide outlines how to set up your development environment, follow project conventions, and submit high-quality contributions to the project.

## 📚 Table of Contents

- [Contributing to MuJoCo Toolbox](#contributing-to-mujoco-toolbox)
  - [📚 Table of Contents](#-table-of-contents)
  - [📜 Code of Conduct](#-code-of-conduct)
  - [⚙️ Getting Started](#️-getting-started)
  - [🚀 How to Contribute](#-how-to-contribute)
  - [🧑‍💻 Development Guidelines](#-development-guidelines)
  - [🔍 Linting and Automation](#-linting-and-automation)
  - [📖 Building the Documentation Locally](#-building-the-documentation-locally)
  - [🐞 Reporting Issues](#-reporting-issues)
  - [📄 License](#-license)

---

## 📜 Code of Conduct

Please review the [Code of Conduct](https://github.com/MGross21/mujoco-toolbox/blob/main/CODE_OF_CONDUCT.md) to ensure a respectful and productive environment for all contributors.

---

## ⚙️ Getting Started

1. **Fork and clone the repository**:

    ```bash
    git clone https://github.com/MGross21/mujoco-toolbox.git
    cd mujoco-toolbox
    ```

2. **Install [uv](https://docs.astral.sh/uv/)** (if you haven't already):

    ```bash
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```

    Or see the [uv installation docs](https://docs.astral.sh/uv/getting-started/installation/).

3. **Create the environment and install everything**:

    ```bash
    uv sync
    ```

    This creates `.venv/`, installs the project in editable mode, and installs
    the `dev` dependency group (which pulls in `test`, `lint`, and `docs`),
    all pinned exactly by `uv.lock`. There is no separate "activate" step —
    prefix commands with `uv run`:

    ```bash
    uv run pytest
    ```

    If you prefer an activated shell, `source .venv/bin/activate` still works.

4. **Optional installs**:

    ```bash
    uv sync --extra examples   # Jupyter kernel + widgets for examples/
    uv sync --group lint       # linters only, without the project
    ```

5. **Changing dependencies**: edit `pyproject.toml`, then run `uv lock` and
    commit the updated `uv.lock`. `uv add <pkg>` / `uv remove <pkg>` do both
    steps for you. **`uv.lock` is committed and must stay in sync** — CI runs
    with `UV_FROZEN=1` and fails if it drifts from `pyproject.toml`.

---

## 🚀 How to Contribute

We welcome:

- 🐛 Bug fixes
- 📚 Documentation improvements
- 🚀 New features
- 🧪 Additional tests
- 🔧 Code refactors

For significant changes, please [open an issue](https://github.com/MGross21/mujoco-toolbox/issues/new) or start a discussion first.

---

## 🧑‍💻 Development Guidelines

- Write clean, modular, and well-documented code.
- Use meaningful names for variables, classes, and functions.
- Avoid large, complex functions — break logic into smaller components.
- Public classes and functions must include Python docstrings.
- Avoid unnecessary dependencies.
- Add or update tests for any new features or bug fixes.

---

## 🔍 Linting and Automation

All pull requests are automatically checked using **GitHub Actions**.

Two tools are enforced:

- [`ruff`](https://docs.astral.sh/ruff/) — linting **and** formatting. Replaces
  black, isort, flake8, bandit, and pylint; all of its config lives in
  `[tool.ruff]` in `pyproject.toml`.
- [`mypy`](https://mypy-lang.org/) — static type checking (ruff does not type-check).

Tests run with [`pytest`](https://docs.pytest.org/).

⚠️ **Your pull request must pass all checks before it can be merged.**

Run everything locally:

```bash
uv run ruff check --fix .     # lint, with autofixes applied
uv run ruff format .          # format
uv run mypy mujoco_toolbox/   # type check
uv run pytest tests/          # tests
```

## 📖 Building the Documentation Locally

To build the documentation locally, run:

```bash
uv run make -C docs html
```

To automatically open the generated documentation in your default web browser after building, use:

```bash
uv run make -C docs html && xdg-open docs/_build/html/index.html
```

This will generate the HTML documentation and open the `index.html` page for easy viewing.

## 🐞 Reporting Issues

Use [GitHub Issues](https://github.com/MGross21/mujoco-toolbox/issues) to:

- Report bugs
- Request features
- Ask usage questions
- Suggest improvements

Please include:

- A clear and descriptive title
- Environment information (OS, Python version, MuJoCo Toolbox version, etc.)
- Reproduction steps (if applicable)
- Screenshots or logs (when helpful)

---

## 📄 License

By contributing, you agree that your work will be released under the
[MIT License](https://github.com/MGross21/mujoco-toolbox/blob/main/LICENSE).
