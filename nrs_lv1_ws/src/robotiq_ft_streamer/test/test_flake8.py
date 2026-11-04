"""Run the standard ament flake8 check."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
@pytest.mark.filterwarnings(
    'ignore:This process .* is multi-threaded.*:DeprecationWarning'
)
def test_flake8():
    """Check Python style."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, '\n'.join(errors)
