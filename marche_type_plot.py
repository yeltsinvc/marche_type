from typing import Sequence

from marche_type import MarcheTypeSimulator, IDMParameters


def plot_example_route(stations: Sequence[float], dwell: float) -> None:
    """Run a simple route simulation and plot the results."""
    params = IDMParameters(v0=15.0, T=1.5, a=0.6, b=1.5, s0=2.0)
    sim = MarcheTypeSimulator(params)
    t, x, v = sim.simulate_route(stations, dwell)
    sim.plot_speed_profiles(t, x, v)


if __name__ == "__main__":
    plot_example_route([0, 300, 800, 1500], 30.0)

