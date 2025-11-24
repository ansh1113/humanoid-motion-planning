from setuptools import setup, find_packages

setup(
    name="humanoid-motion-planning",
    version="0.1.0",
    description="Humanoid whole-body motion planning with ZMP constraints",
    author="Ansh Bhansali",
    author_email="anshbhansali5@gmail.com",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "matplotlib>=3.4.0",
    ],
    extras_require={
        "drake": ["pydrake"],  # Optional, for full simulation
        "dev": [
            "pytest>=6.0",
            "pytest-cov>=2.0",
            "black>=21.0",
            "flake8>=3.9",
        ]
    },
    python_requires=">=3.8",
)
