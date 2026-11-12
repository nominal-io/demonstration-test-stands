import pytest

from fluids_panel.mock import MockBackend, MockParams


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def mock(clock: FakeClock) -> MockBackend:
    """A noiseless simulated panel on the fake clock. Not opened."""
    return MockBackend(MockParams(noise_fraction=0.0), clock=clock)


