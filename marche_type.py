import math
from dataclasses import dataclass
from typing import List, Tuple

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

def simulate_marche_type(
    lead_init: VehicleState,
    follower_init: VehicleState,
    params: IDMParameters,
    dt: float,
    steps: int,
    lead_speed: float,
) -> List[Tuple[float, float]]:
    """Simulate follower using IDM. Returns list of (distance, speed)."""
    lead = VehicleState(lead_init.x, lead_init.v)
    follower = VehicleState(follower_init.x, follower_init.v)
    result = []
    for _ in range(steps):
        lead.v = lead_speed  # keep lead vehicle at constant speed
        lead.x += lead.v * dt

        a = idm_acceleration(lead, follower, params)
        follower.v = max(0.0, follower.v + a * dt)
        follower.x += follower.v * dt
        result.append((follower.x, follower.v))
    return result

if __name__ == "__main__":
    params = IDMParameters(v0=15.0, T=1.5, a=0.6, b=1.5, s0=2.0)
    lead_init = VehicleState(x=50.0, v=10.0)
    follower_init = VehicleState(x=0.0, v=0.0)
    dt = 0.5  # 0.5 second step
    steps = 120  # simulate 1 minute
    lead_speed = 10.0

    data = simulate_marche_type(lead_init, follower_init, params, dt, steps, lead_speed)
    for distance, speed in data:
        print(f"{distance:.2f}, {speed:.2f}")
