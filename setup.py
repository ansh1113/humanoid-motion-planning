from setuptools import setup, find_packages
from pathlib import Path

# Read the contents of README file
this_directory = Path(__file__).parent.resolve()
with open(this_directory / 'README.md', encoding='utf-8') as f:
    long_description = f.read()

# Read requirements
with open(this_directory / 'requirements.txt', encoding='utf-8') as f:
    requirements = [
        line.split(';')[0].strip() for line in f 
        if line.strip() and not line.startswith('#')
    ]
    # Remove empty strings from splitting
    requirements = [req for req in requirements if req]

setup(
    name="humanoid-motion-planning",
    version="0.1.0",
    description="Humanoid whole-body motion planning with ZMP constraints for safe reaching tasks",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Ansh Bhansali",
    author_email="anshbhansali5@gmail.com",
    url="https://github.com/ansh1113/humanoid-motion-planning",
    project_urls={
        "Bug Tracker": "https://github.com/ansh1113/humanoid-motion-planning/issues",
        "Documentation": "https://github.com/ansh1113/humanoid-motion-planning/blob/main/README.md",
        "Source Code": "https://github.com/ansh1113/humanoid-motion-planning",
    },
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=requirements,
    extras_require={
        "drake": ["pydrake"],  # Optional, for full simulation
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "pytest-mock>=3.6",
            "black>=21.0",
            "flake8>=3.9",
            "mypy>=0.910",
            "pre-commit>=2.15",
        ],
        "viz": [
            "pyqt5>=5.15",
            "pyqtgraph>=0.12",
        ],
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
        "Operating System :: OS Independent",
    ],
    keywords="robotics motion-planning humanoid zmp balance trajectory-optimization",
    license="MIT",
    include_package_data=True,
    zip_safe=False,
)
