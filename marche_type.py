"""Core library for simulating a BRT marche type using IDM."""

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

try:  # optional dependency for plotting
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - matplotlib may not be installed
    plt = None

@dataclass
class VehicleState:
    x: float  # position (m)
    v: float  # velocity (m/s)

@dataclass
class IDMParameters:
    v0: float  # desired speed (m/s)
    T: float   # safe time headway (s)
    a: float   # max acceleration (m/s^2)
    b: float   # comfortable deceleration (m/s^2)
    s0: float  # minimum gap (m)
    delta: int = 4  # acceleration exponent

def idm_acceleration(lead: VehicleState, follower: VehicleState, params: IDMParameters) -> float:
    """Compute acceleration for the follower using the Intelligent Driver Model."""
    s = lead.x - follower.x  # gap to lead vehicle
    if s <= 0:
        return -params.b
    dv = follower.v - lead.v
    s_star = params.s0 + max(0.0, follower.v * params.T + (follower.v * dv) / (2 * math.sqrt(params.a * params.b)))
    return params.a * (1 - (follower.v / params.v0) ** params.delta - (s_star / s) ** 2)


class MarcheTypeSimulator:
    """Encapsulates IDM-based simulations and optional plotting."""

    def __init__(self, params: IDMParameters, dt: float = 0.5) -> None:
        self.params = params
        self.dt = dt

    def simulate_idm(
        self,
        lead_init: VehicleState,
        follower_init: VehicleState,
        steps: int,
        lead_speed: float,
    ) -> List[Tuple[float, float]]:
        """Simulate follower using IDM. Returns list of (distance, speed)."""
        lead = VehicleState(lead_init.x, lead_init.v)
        follower = VehicleState(follower_init.x, follower_init.v)
        result: List[Tuple[float, float]] = []
        for _ in range(steps):
            lead.v = lead_speed
            lead.x += lead.v * self.dt

            a = idm_acceleration(lead, follower, self.params)
            follower.v = max(0.0, follower.v + a * self.dt)
            follower.x += follower.v * self.dt
            result.append((follower.x, follower.v))
        return result

    def simulate_route(
        self,
        stations: Sequence[float],
        dwell_time: float,
        accel: float = 0.7,
        decel: float = 0.7,
        v_max: float = 20.0,
    ) -> Tuple[List[float], List[float], List[float]]:
        """Generate a speed profile for a route with stops."""
        time = [0.0]
        pos = [stations[0]]
        vel = [0.0]
        t = 0.0
        x = stations[0]
        v = 0.0
        for idx in range(1, len(stations)):
            target = stations[idx]
            while x < target:
                dist_left = target - x
                stopping_dist = v ** 2 / (2 * decel)
                if dist_left <= stopping_dist:
                    v = max(0.0, v - decel * self.dt)
                else:
                    if v < v_max:
                        v = min(v_max, v + accel * self.dt)
                x += v * self.dt
                t += self.dt
                time.append(t)
                pos.append(x)
                vel.append(v)
            x = target
            time.append(t)
            pos.append(x)
            vel.append(0.0)
            for _ in range(int(dwell_time / self.dt)):
                t += self.dt
                time.append(t)
                pos.append(x)
                vel.append(0.0)
            v = 0.0
        return time, pos, vel

    def plot_speed_profiles(
        self, time: Iterable[float], pos: Iterable[float], vel: Iterable[float]
    ) -> None:
        """Plot speed-distance and speed-time graphs if matplotlib is available."""
        if plt is None:
            raise ImportError("matplotlib is required for plotting")
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(pos, vel)
        plt.xlabel("Distancia (m)")
        plt.ylabel("Velocidad (m/s)")
        plt.title("Velocidad vs Distancia")

        plt.subplot(2, 1, 2)
        plt.plot(time, vel)
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Velocidad (m/s)")
        plt.title("Velocidad vs Tiempo")
        plt.tight_layout()
        plt.show()

def simulate_marche_type(
    lead_init: VehicleState,
    follower_init: VehicleState,
    params: IDMParameters,
    dt: float,
    steps: int,
    lead_speed: float,
) -> List[Tuple[float, float]]:
    """Backward compatible helper that delegates to :class:`MarcheTypeSimulator`."""
    simulator = MarcheTypeSimulator(params, dt)
    return simulator.simulate_idm(
        lead_init=lead_init,
        follower_init=follower_init,
        steps=steps,
        lead_speed=lead_speed,
    )

if __name__ == "__main__":
    params = IDMParameters(v0=15.0, T=1.5, a=0.6, b=1.5, s0=2.0)
    lead_init = VehicleState(x=50.0, v=10.0)
    follower_init = VehicleState(x=0.0, v=0.0)
    dt = 0.5  # 0.5 second step
    steps = 120  # simulate 1 minute
    lead_speed = 10.0

    simulator = MarcheTypeSimulator(params, dt)
    data = simulator.simulate_idm(
        lead_init=lead_init,
        follower_init=follower_init,
        steps=steps,
        lead_speed=lead_speed,
    )
    for distance, speed in data:
        print(f"{distance:.2f}, {speed:.2f}")
