import logging
import copy
from collections import defaultdict
from pathlib import Path
import argparse
import pickle
from OAKDLogger import OAKDLogger
import numpy as np
import cv2


def parse_args():
    """
    Parse arguments
    """
    parser = argparse.ArgumentParser(description='OAK-D Logger')

    parser.add_argument('logdir', help='Path where logs are written', type=str)
    parser.add_argument('--output', help='Output binary file name', type=str, default=None)
    parser.add_argument('--input', help='Input binary file name. If specified replay file', type=str, default=None)
    parser.add_argument('--sequential', help='If specified, read input file sequentially', action='store_true')
    parser.add_argument('--pickle', help='File name where input binary file is pickled', type=str, default=None)
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
        sequential_replay = args.sequential or (args.pickle is not None)
        if not sequential_replay:
            logger.replay(args.input)
        else:
            cam_packet = dict(img=np.ascontiguousarray(np.zeros((720, 1280), dtype=np.uint8)), timestamp=0.0)
            imu_packet = dict(accelerometer=np.zeros(3), gyroscope=np.zeros(3), timestamp=0.0)
            pickle_data = defaultdict(list)
            num_packets = defaultdict(int)
            while True:
                data_type = logger.sequential_read(args.input, cam_packet, imu_packet)
                if data_type == data_type.INVALID:
                    break

                num_packets[data_type.name] += 1
                if args.pickle:
                    type_name = data_type.name
                    pickle_data['manifest'].append(type_name)
                    cam_type = (data_type == data_type.LEFT_MONO or
                                data_type == data_type.RIGHT_MONO or
                                data_type == data_type.RGB)
                    if data_type == data_type.IMU:
                        pickle_data[type_name].append(copy.deepcopy(imu_packet))
                    elif cam_type:
                        pickle_data[type_name].append(copy.deepcopy(cam_packet))
            print("Done with sequential read with: ")
            for data_type, num_packet in num_packets.items():
                print(f"{data_type} : {num_packet}")

            if args.pickle:
                with open(args.pickle, 'wb') as f:
                    pickle.dump(pickle_data, f)
                    print(f"Wrote sequential read to {args.pickle}")


if __name__ == '__main__':
    main()
