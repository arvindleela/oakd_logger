import sys
sys.path.insert(0, "build")

from pathlib import Path
import argparse
from OAKDLogger import OAKDLogger

def parse_args():
    """
    Parse arguments
    """
    parser = argparse.ArgumentParser(description='OAK-D Logger')

    parser.add_argument('logdir', help='Path where logs are written', type=str)
    parser.add_argument('--duration', help='Duration of logging, s', type=float, default=1.0)
    return vars(parser.parse_args())

def main():
    args = parse_args()
    logger = OAKDLogger(args)

    if not logger.initialize():
        raise Exception('Logger initialize failed.')

    # Start logging
    logger.start_logging()

if __name__ == '__main__':
    main()

