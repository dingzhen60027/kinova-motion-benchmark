"""Private subprocess entry point; scheduling is owned by the public CLI."""
import argparse
from .runner import worker


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--job", required=True)
    parser.add_argument("--attempt", required=True)
    args = parser.parse_args()
    worker(args.run, args.job, args.attempt)
