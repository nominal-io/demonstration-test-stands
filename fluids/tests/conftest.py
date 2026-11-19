import pytest

from fluids_panel.mock import MockBackend, MockParams
from fluids_panel.stand import FluidsStand


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


@pytest.fixture
def stand(mock: MockBackend, clock: FakeClock):
    """An opened stand on the simulated panel."""
    with FluidsStand(mock, clock=clock) as stand:
        yield stand


@pytest.fixture
def noisy_stand(clock: FakeClock):
    """An opened stand on the simulated panel with its default (seeded) sensor noise, as in Connect."""
    with FluidsStand(
        MockBackend(clock=clock, seed=11, noise=True), clock=clock
    ) as stand:
        yield stand


def tick(stand: FluidsStand, clock: FakeClock, seconds: float, dt: float = 0.1) -> None:
    """Run the stand's update loop for `seconds` of fake time."""
    for _ in range(round(seconds / dt)):
        clock.now += dt
        stand.update()


def tick_until(
    stand: FluidsStand,
    clock: FakeClock,
    condition,
    timeout_s: float,
    dt: float = 0.1,
) -> None:
    """Run the update loop until `condition()` holds, failing the test after `timeout_s`."""
    deadline = clock.now + timeout_s
    while not condition():
        assert clock.now < deadline, "condition not reached in time"
        clock.now += dt
        stand.update()
