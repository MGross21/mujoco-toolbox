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

2. **Install `uv`** (if you haven’t already):

    ```bash
    pip install uv
    ```

3. **Install Code as `Dev`**

    ```bash
    uv sync --group dev --color=always
    ```

4. **Running Rust in Develop Mode**

    ```dev
    uv run maturin develop
    ```

    *If `libmujoco.so` isn't found, run*

    ```bash
    export LD_LIBRARY_PATH=$(python -c "import mujoco; print(mujoco.__path__[0])"):$LD_LIBRARY_PATH
    ```

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

The following tools are enforced:

- [`black`](https://black.readthedocs.io/) – formatting
- [`ruff`](https://docs.astral.sh/ruff/) – linting
- [`mypy`](http://mypy-lang.org/) – type checking
- [`pytest`](https://docs.pytest.org/) – test runner

⚠️ **Your pull request must pass all checks before it can be merged.**

Run linter locally:

```bash
uv run ruff format mujoco_toolbox
```

## 📖 Building the Documentation Locally

To build the documentation locally, run:

```bash
uv run sphinx-build docs docs/_build/html
```

To automatically open the generated documentation in your default web browser after building, use:

```bash
uv run sphinx-build docs docs/_build/html && start docs/_build/html/index.html
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
