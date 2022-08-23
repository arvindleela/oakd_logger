import sys
sys.path.insert(0, "build")

import logging
from pathlib import Path
import argparse
from OAKDLogger import OAKDLogger

def parse_args():
    """
    Parse arguments
    """
    parser = argparse.ArgumentParser(description='OAK-D Logger')

    parser.add_argument('logdir', help='Path where logs are written', type=str)
    parser.add_argument('--output', help='Output binary file name', type=str, default=None)
    parser.add_argument('--input', help='Input binary file name. If specified replay file', type=str, default=None)
    return parser.parse_args()

def main():
    args = parse_args()
    logger = OAKDLogger(vars(args))

    if args.input is None:
        # In logging mode
        logging.info(f"In logging mode ...")
        if args.output is not None:
            op_file = Path(args.logdir) / args.output
            if not logger.prepare_output_stream(op_file.as_posix()):
                raise Exception('Unable to prepare output log')

        if not logger.initialize():
            raise Exception('Logger initialize failed.')

        # Start logging
        logger.start_logging()
    else:
        logging.info(f"In replay mode ...")
        logger.replay(args.input)

if __name__ == '__main__':
    main()

