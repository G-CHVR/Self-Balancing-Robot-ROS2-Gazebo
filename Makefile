# Makefile for Self-Balancing Robot Project

# Variables
ROS_DISTRO ?= jazzy
SRC_DIRS := src/
PYTHON_SOURCES := $(shell find $(SRC_DIRS) -name '*.py')
CPP_SOURCES := $(shell find $(SRC_DIRS) -name '*.cpp' -o -name '*.hpp')
BUILD_DIR := build/
INSTALL_DIR := install/
LOG_DIR := log/

# Default target
.PHONY: all
all: lint build test

# Linting
.PHONY: lint
lint: lint-python lint-cpp

.PHONY: lint-python
lint-python:
	@echo "Running Python linting..."
	@which flake8 > /dev/null || (echo "flake8 not found, installing..." && pip3 install flake8 flake8-docstrings)
	flake8 $(SRC_DIRS) --count --select=E9,F63,F7,F82 --show-source --statistics
	flake8 $(SRC_DIRS) --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics

.PHONY: lint-cpp
lint-cpp:
	@echo "Running C++ linting..."
	@which clang-format > /dev/null || (echo "clang-format not found, installing..." && sudo apt-get install -y clang-format)
	@echo "Checking C++ files with clang-format..."
	@find $(SRC_DIRS) -name '*.cpp' -o -name '*.hpp' | xargs clang-format -style=file -output-replacements-xml | grep -c "<replacement " && echo "Code style issues found." && exit 1 || echo "No code style issues found."

# Build
.PHONY: build
build:
	@echo "Building ROS 2 packages..."
	colcon build --symlink-install

# Test
.PHONY: test
test:
	@echo "Running tests..."
	colcon test
	colcon test-result --verbose

# Clean
.PHONY: clean
clean:
	@echo "Cleaning build, install, and log directories..."
	rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)

# Help
.PHONY: help
help:
	@echo "Usage:"
	@echo "  make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all          Runs lint, build, and test (default)"
	@echo "  lint         Runs both Python and C++ linters"
	@echo "  lint-python  Runs Python linter (flake8)"
	@echo "  lint-cpp     Runs C++ linter (clang-format)"
	@echo "  build        Builds the ROS 2 packages"
	@echo "  test         Runs the tests"
	@echo "  clean        Cleans the build, install, and log directories"
	@echo "  help         Shows this help message"
