import argparse
from marche_type import IDMParameters, VehicleState, MarcheTypeSimulator


def main() -> None:
    parser = argparse.ArgumentParser(description="Run marche type simulations")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="plot a sample route with stops",
    )
    args = parser.parse_args()

    params = IDMParameters(v0=15.0, T=1.5, a=0.6, b=1.5, s0=2.0)
    sim = MarcheTypeSimulator(params)

    if args.plot:
        t, x, v = sim.simulate_route([0, 300, 800, 1500], dwell_time=30.0)
        try:
            sim.plot_speed_profiles(t, x, v)
        except ImportError as exc:
            print("Plotting unavailable:", exc)
    else:
        data = sim.simulate_idm(
            lead_init=VehicleState(x=50.0, v=10.0),
            follower_init=VehicleState(x=0.0, v=0.0),
            steps=120,
            lead_speed=10.0,
        )
        for distance, speed in data:
            print(f"{distance:.2f}, {speed:.2f}")


if __name__ == "__main__":
    main()
