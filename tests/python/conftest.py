"""
JaguarEngine Python Test Configuration

pytest configuration for pyjaguar tests.
"""

import pytest
import sys
import os

# Add build directory to path for finding pyjaguar module
def pytest_configure(config):
    """Configure pytest for pyjaguar testing."""
    # Common build directories where pyjaguar.*.so might be located
    build_paths = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'build'),
        os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'Release'),
        os.path.join(os.path.dirname(__file__), '..', '..', 'build', 'Debug'),
        os.path.join(os.path.dirname(__file__), '..', '..', 'cmake-build-release'),
        os.path.join(os.path.dirname(__file__), '..', '..', 'cmake-build-debug'),
    ]

    for path in build_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path) and abs_path not in sys.path:
            sys.path.insert(0, abs_path)


def pytest_collection_modifyitems(config, items):
    """Mark tests that require pyjaguar."""
    try:
        import pyjaguar
    except ImportError:
        skip_marker = pytest.mark.skip(reason="pyjaguar module not available (build with JAGUAR_BUILD_PYTHON=ON)")
        for item in items:
            if "pyjaguar" in item.name or "test_" in item.name:
                item.add_marker(skip_marker)


@pytest.fixture(scope="session")
def pyjaguar_available():
    """Check if pyjaguar is available."""
    try:
        import pyjaguar
        return True
    except ImportError:
        return False


@pytest.fixture
def engine():
    """Provide a fresh Engine instance for each test."""
    try:
        import pyjaguar as jag
        eng = jag.Engine()
        eng.initialize()
        yield eng
        eng.shutdown()
    except ImportError:
        pytest.skip("pyjaguar not available")
