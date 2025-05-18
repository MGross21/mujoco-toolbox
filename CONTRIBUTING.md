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

2. **Install Poetry** (if you haven’t already):

    ```bash
    curl -sSL https://install.python-poetry.org | python3 -
    ```

    Or see: [Poetry Installation Docs](https://python-poetry.org/docs/#installation)

3. **Install dependencies**:

    ```bash
    poetry install --with dev
    ```

4. **Activate the shell**:

    ```bash
    poetry shell
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

Run checks locally using Poetry:

```bash
poetry run black mujoco_toolbox
poetry run ruff check mujoco_toolbox
poetry run mypy mujoco_toolbox
poetry run pytest tests/
```

## 🐞 Reporting Issues

Use [GitHub Issues](https://github.com/MGross21/mujoco-toolbox/issues) to:

- Report bugs
- Request features
- Ask usage questions
- Suggest improvements

Please include:

- A clear and descriptive title
- Environment information (OS, Python version, MuJoCo version, etc.)
- Reproduction steps (if applicable)
- Screenshots or logs (when helpful)

---

## 📄 License

By contributing, you agree that your work will be released under the
[MIT License](https://github.com/MGross21/mujoco-toolbox/blob/main/LICENSE).
