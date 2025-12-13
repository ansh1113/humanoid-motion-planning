# Use official Python runtime as base image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Install development dependencies
RUN pip install --no-cache-dir \
    pytest \
    pytest-cov \
    black \
    flake8 \
    jupyter \
    ipython

# Copy the rest of the application
COPY . .

# Install the package
RUN pip install -e .

# Create directories for data and results
RUN mkdir -p /app/data /app/results /app/logs

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/app/src:$PYTHONPATH

# Expose port for Jupyter
EXPOSE 8888

# Default command
CMD ["/bin/bash"]
