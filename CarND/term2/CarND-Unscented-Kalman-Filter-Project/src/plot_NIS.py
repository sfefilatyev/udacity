"""Plots NIS graph according to NIS values and Chi-squared distribution degrees of freedom"""

import argparse
import logging
import matplotlib.pyplot as plt
import os

# The following table shows 5-percentile value for various degrees of freedom.
# The table was taken from UKF lesson - see video from #31.
CHI_SQUARED_LOOKUP_TABLE = {
    "df": {
        1: 3.841,
        2: 5.991,
        3: 7.815,
        4: 9.488,
        5: 11.070 
        }
    }

def main(args=None):
    parser = argparse.ArgumentParser(
        description="Tool to plot NIS graph for Radar/Lidar NIS values.")
    parser.add_argument(
        '-f',
        '--file',
        required=True,
        help='Input file with NIS value (one value per line).'
        )
    parser.add_argument(
        '-df',
        '--degrees_freedom',
        type=int,
        required=True,
        help='Number of degrees of freedom for measurement vector (Radar=3, Lidar=2).'
        )
    parser.add_argument(
        '-t',
        '--title',
        help='Optional caption to use in the plot.')
    args = parser.parse_args(args)

    assert os.path.exists(args.file), 'Input file {} does not exist!'.format(args.file)
    assert int(args.degrees_freedom) >= 1 and  int(args.degrees_freedom) <= 5, \
        "Degrees of Freedom should be between 1 and 5."
    with open(args.file) as nis_file:
        lines = nis_file.readlines()
        nis_values = [float(line.strip()) for line in lines]
        x_indices = [x for x in range(len(nis_values))]
        nis_threshold = [CHI_SQUARED_LOOKUP_TABLE["df"][args.degrees_freedom] for _ in range(len(nis_values))]
        plt.switch_backend('qt4agg')
        fig = plt.figure()
        #ax = plt.axes()
        plt.plot(x_indices, nis_threshold, '--r', label='Threshold')
        plt.plot(x_indices, nis_values, label='NIS values')
        plt.xlabel("Measurements")
        plt.ylabel("Measurements values")
        if args.title:
            plt.title(args.title)
        plt.pause(20)

if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        level=logging.INFO)
    main()