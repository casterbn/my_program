import pandas as pd
import os
import sys
from matplotlib import pyplot as plt


def label_trajectory(fpath):
    traj = pd.read_csv(fpath, sep = ',', header = None)
    traj.columns = ['time', 'lon', 'lat', 'hgt', 'heading', 'label', 'int']
    traj.drop('int', axis = 1)

    lon, lat = traj['lon'], traj['lat']
    plt.figure()
    plt.plot(lon, lat, 'b.-')
    plt.show()

    t, hdg = traj['time'], traj['heading']
    plt.figure()
    plt.plot(t, hdg, 'b.-')
    plt.show()


def main():
    if len(sys.argv) < 2:
        print('Usage: python.exe *.py file_path_to_input_data.txt')
        file_path = raw_input('Input the file path: ')
    else:
        file_path = sys.argv[1]


    label_trajectory(file_path)
    print 'Done'


if __name__ == '__main__':
    main()
