import datetime
import math
import serial
import serial.tools.list_ports
from multiprocessing import Process, Pipe, Array
import time
import struct
import numpy as np
# import attitude
# import openimu
import imu38x
# import ins1000
import dynamic_kml as kml
# import post_proccess_for_ins_test

#### INS381
ins381_unit = {'port': '/dev/tty.usbserial-1413100',\
            'baud':115200,\
            'packet_type':'k1',\
            'unit_type':'imu38x',\
            # 'orientation':'-y+x+z',\
            'enable':True}
#### INS1000
ins1000_unit = {'port':'COM15',\
                'baud':230400,\
                'packet_type':'nav',\
                'unit_type':'ins1000',\
                'enable':False}

log_dir = './log_data/'
log_file = 'log_' + datetime.datetime.now().strftime("%b_%d_%Y_%H_%M_%S") + '.csv'

enable_kml = False


def log_imu38x(port, baud, packet, pipe):
    imu38x_unit = imu38x.imu38x(port, baud, packet_type=packet, pipe=pipe)
    imu38x_unit.start()

def log_ins1000(port, baud, pipe):
    ins = ins1000.ins1000(port, baud, pipe)
    ins.start()


def orientation(data, ori):
    '''
    change coordiante according to ori.
    Args:
        data: nx3 numpy array
        ori: a string including +x, -x +y, -y, +z, -z, for example: '-y+x+z'
    Return:
        data after coordinate change
    '''
    map = {'+x': [0, 1],\
           '-x': [0, -1],\
           '+y': [1, 1],\
           '-y': [1, -1],\
           '+z': [2, 1],\
           '-z': [2, -1]}
    ori = ori.lower()
    idx_x = int(0)
    sgn_x = 1
    idx_y = int(1)
    sgn_y = 1
    idx_z = int(2)
    sgn_z = 1
    if len(ori) == 6:
        if ori[0:2] in map:
            idx_x = int(map[ori[0:2]][0])
            sgn_x = map[ori[0:2]][1]
        if ori[2:4] in map:
            idx_y = int(map[ori[2:4]][0])
            sgn_y = map[ori[2:4]][1]
        if ori[4:6] in map:
            idx_z = int(map[ori[4:6]][0])
            sgn_z = map[ori[4:6]][1]
        data[0], data[1], data[2] =\
            sgn_x * data[idx_x], sgn_y * data[idx_y], sgn_z * data[idx_z]
    return data

if __name__ == "__main__":
    #### find ports
    if not ins381_unit['enable']:
        ins381_unit['port'] = None
    if not ins1000_unit['enable']:
        ins1000_unit['port'] = None
    print('%s is an INS381.' % ins381_unit['port'])
    print('%s is an INS1000.' % ins1000_unit['port'])

    #### create pipes
    # ins381
    if ins381_unit['enable']:
        print('connecting to the unti with NXP accel...')
        parent_conn_nxp, child_conn_nxp = Pipe()
        process_target = log_imu38x
        p_nxp = Process(target=process_target,\
                        args=(ins381_unit['port'], ins381_unit['baud'],\
                              ins381_unit['packet_type'], child_conn_nxp)
                       )
        p_nxp.daemon = True
        p_nxp.start()

    # ins1000
    if ins1000_unit['enable']:
        print('connecting to INS1000...')
        parent_conn_ins1000, child_conn_ins1000 = Pipe()
        p_ins1000 = Process(target=log_ins1000,\
                            args=(ins1000_unit['port'], ins1000_unit['baud'],\
                                  child_conn_ins1000)
                           )
        p_ins1000.daemon = True
        p_ins1000.start()

    #### create log file
    data_file = log_dir + log_file
    f = open(data_file, 'w+')
    f.truncate()
    headerline = "recv_interval (s), openimu timer, gps_tow (ms),"
    headerline += "Lat (deg), Lon (deg), Alt (m),"
    headerline += "Lat_base (deg), Lon_base (deg), Alt_base (m),"
    headerline += "vN (m/s), vE (m/s), vD (m/s),"
    headerline += "hdop, hor_acc, ver_acc, sol_age,"
    headerline += "nav_status,num_psr, num_adr\n"
    f.write(headerline)
    f.flush()

    #### start logging
    # start time, to calculate recv interval
    tstart = time.time()
    # data from INS381
    ins381_timer = 0
    ins381_acc = np.zeros((3,))
    ins381_gyro = np.zeros((3,))
    ins381_lla = np.zeros((3,))
    ins381_vel = np.zeros((3,))
    ins381_euler = np.zeros((3,))
    # data from ins1000
    ref_lla = np.zeros((3,))
    ref_vel = np.zeros((3,))
    ref_euler = np.zeros((3,))
    # logging
    counter = 0
    try:
        while True:
            # 1. timer interval
            tnow = time.time()
            time_interval = tnow-tstart
            tstart = tnow
            # 2. INS381, timer, acc and gyro, lla, vel, Euler angles
            if ins381_unit['enable']:
                latest_ins381 = parent_conn_nxp.recv()
                ins381_timer = latest_ins381[0]
                gps_tow = latest_ins381[1]
                rtk_llh = np.array(latest_ins381[2])
                base_llh = np.array(latest_ins381[3])
                rtk_vel = np.array(latest_ins381[4])
                rtk_acc = np.array(latest_ins381[5]) # accuracy measures
                rtk_qua = np.array(latest_ins381[6])
                # if 'orientation' in ins381_unit:
                    # ins381_acc = orientation(ins381_acc, ins381_unit['orientation'])
                    # ins381_gyro = orientation(ins381_gyro, ins381_unit['orientation'])
            # 3. ins1000 time, lla, vel and quat
            if ins1000_unit['enable']:
                # ins1000 can be of higher sampling rate, get the latest one
                latest_ref = None
                while parent_conn_ins1000.poll():
                    latest_ref = parent_conn_ins1000.recv()
                if latest_ref is not None:
                    ref_lla = np.array(latest_ref[1])
                    ref_vel = np.array(latest_ref[2])
                    ref_quat = np.array(latest_ref[3])
                    try:
                        ref_euler = attitude.quat2euler(ref_quat)   #ypr
                    except:
                        print("quat: %s"% latest_ref[3])
                    ref_euler[0] = ref_euler[0] * attitude.R2D
                    ref_euler[1] = ref_euler[1] * attitude.R2D
                    ref_euler[2] = ref_euler[2] * attitude.R2D
            else:
                print('logging...')
                # tmp = latest_ins381[8]
                # # ref_lla = np.array([tmp[0], tmp[1], 0.0])
                # # ref_euler[0] = tmp[2]
                # ref_lla = np.array([tmp[0], tmp[1], tmp[2]])
                # tmp = latest_ins381[3]; # DW DEBUG, mag data holding ref_vel
                # ref_vel = np.array([tmp[0],tmp[1], tmp[2]])
            # 5. log data to file
            fmt = "%f, %u, %u, "                    # time_interval, packet timer
            fmt += "%.9f, %.9f, %.9f, "
            fmt += "%.9f, %.9f, %.9f, " # ins381 lla/vel/euler
            fmt += "%f, %f, %f, %f, %f, %f, %f, "
            fmt += "%u, %u, %u\n" # ref lla/vel/euler
            lines = fmt% (\
                            time_interval, ins381_timer, gps_tow,\
                            rtk_llh[0], rtk_llh[1], rtk_llh[2],\
                            base_llh[0], base_llh[1], base_llh[2],\
                            rtk_vel[0], rtk_vel[1], rtk_vel[2],\
                            rtk_acc[0], rtk_acc[1], rtk_acc[2], rtk_acc[3],\
                            rtk_qua[0], rtk_qua[1], rtk_qua[2])
            f.write(lines)
            f.flush()
            counter += 1
            if enable_kml and counter == 10:
                counter = 0
                kml.gen_kml('./kml/ins381.kml', ins381_lla, ins381_euler[2], 'ffff0000')
                # kml.gen_kml('./kml/ins1000.kml', ref_lla, ref_euler[0], 'ff0000ff')
    except KeyboardInterrupt:
        print("Stop logging, preparing data for simulation...")
        f.close()
        if ins381_unit['enable']:
            p_nxp.terminate()
            p_nxp.join()
        if ins1000_unit['enable']:
            p_ins1000.terminate()
            p_ins1000.join()
        # post_proccess_for_ins_test.post_processing(data_file)
