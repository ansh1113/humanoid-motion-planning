# Contributing to Humanoid Motion Planning

First off, thank you for considering contributing to this project! 🎉

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [How to Contribute](#how-to-contribute)
- [Style Guidelines](#style-guidelines)
- [Testing](#testing)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)

## Code of Conduct

This project and everyone participating in it is governed by our Code of Conduct. By participating, you are expected to uphold this code.

## Getting Started

1. Fork the repository on GitHub
2. Clone your fork locally:
   ```bash
   git clone https://github.com/your-username/humanoid-motion-planning.git
   cd humanoid-motion-planning
   ```

## Development Setup

### Using Python Virtual Environment

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
pip install -e ".[dev]"

# Install pre-commit hooks
pip install pre-commit
pre-commit install
```

### Using Docker

```bash
# Build and run development container
docker-compose up -d dev
docker-compose exec dev bash
```

## How to Contribute

### Reporting Bugs

Before creating bug reports, please check existing issues. When you create a bug report, include as many details as possible:

- Use a clear and descriptive title
- Describe the exact steps to reproduce the problem
- Provide specific examples
- Describe the behavior you observed and what you expected
- Include code samples and error messages
- Note your environment (OS, Python version, etc.)

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. When creating an enhancement suggestion:

- Use a clear and descriptive title
- Provide a detailed description of the proposed feature
- Explain why this enhancement would be useful
- Include code examples if applicable

### Code Contributions

1. **Find or create an issue** - Make sure there's an issue for what you're working on
2. **Create a branch** - Branch from `develop` (not `main`):
   ```bash
   git checkout -b feature/your-feature-name develop
   ```
3. **Make your changes** - Follow the style guidelines
4. **Write tests** - Ensure your changes are tested
5. **Run tests** - Make sure all tests pass:
   ```bash
   pytest tests/ -v
   ```
6. **Commit your changes** - Use clear commit messages:
   ```bash
   git commit -m "Add feature: brief description"
   ```
7. **Push and create PR** - Push to your fork and create a pull request

## Style Guidelines

### Python Style

We follow PEP 8 with some modifications:

- Line length: 100 characters
- Use Black for formatting
- Use type hints where possible
- Write docstrings for all public functions/classes (Google style)

```python
def compute_zmp(forces: np.ndarray, positions: np.ndarray) -> np.ndarray:
    """
    Compute Zero Moment Point from ground reaction forces.
    
    Args:
        forces: Ground reaction forces (N x 3)
        positions: Contact positions (N x 3)
        
    Returns:
        zmp: Zero Moment Point (2,)
    """
    # Implementation
```

### Code Formatting

Run these before committing:

```bash
# Format code
black src/ tests/

# Check linting
flake8 src/ tests/

# Sort imports
isort src/ tests/
```

Or use pre-commit:

```bash
pre-commit run --all-files
```

## Testing

### Writing Tests

- Place tests in `tests/` directory
- Name test files `test_*.py`
- Use descriptive test names: `test_compute_zmp_with_equal_forces`
- Test edge cases and error conditions
- Aim for >90% code coverage

### Running Tests

```bash
# Run all tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=humanoid_planner --cov-report=html

# Run specific test file
pytest tests/test_zmp_constraint.py -v

# Run specific test
pytest tests/test_zmp_constraint.py::TestZMPConstraint::test_compute_zmp -v
```

## Documentation

### Code Documentation

- Use Google-style docstrings
- Document all parameters, return values, and exceptions
- Include examples for complex functions

### README and Docs

- Update README.md if you add features
- Add algorithm documentation in `docs/`
- Include usage examples

## Pull Request Process

1. **Update documentation** - Ensure README and docs are updated
2. **Add tests** - New features must have tests
3. **Pass CI** - All CI checks must pass
4. **Update CHANGELOG** - Add your changes to CHANGELOG.md
5. **Request review** - Request review from maintainers
6. **Address feedback** - Make requested changes
7. **Squash commits** - Keep history clean (if requested)

### PR Title Format

Use conventional commits format:

- `feat: Add new trajectory optimizer`
- `fix: Correct ZMP calculation bug`
- `docs: Update API documentation`
- `test: Add tests for kinematics`
- `refactor: Improve support polygon computation`
- `chore: Update dependencies`

### PR Description Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
Describe testing performed

## Checklist
- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments added for complex code
- [ ] Documentation updated
- [ ] Tests added/updated
- [ ] All tests pass
- [ ] No breaking changes (or documented)
```

## Project Structure

```
humanoid-motion-planning/
├── src/humanoid_planner/    # Main package
│   ├── motion_planner.py    # Core motion planning
│   ├── zmp_constraint.py    # ZMP constraints
│   ├── kinematics.py        # FK/IK solvers
│   ├── perception.py        # Perception modules
│   └── ...
├── tests/                   # Test files
├── config/                  # Configuration files
├── scripts/                 # Example scripts
├── docs/                    # Documentation
└── examples/                # Example notebooks
```

## Recognition

Contributors will be recognized in:
- README.md contributors section
- Release notes
- GitHub contributors page

## Questions?

Feel free to:
- Open an issue for questions
- Join discussions in GitHub Discussions
- Contact maintainers

Thank you for contributing! 🚀
