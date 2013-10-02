#Copyright 2013 Google Inc. All Rights Reserved.
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

import time, sys
import struct
import serial
import datetime
#import curses

class IMUInterface():
    def __init__(self, port):
        self.serial = serial.Serial(port, 921600, timeout=1)
        self.cal_fmt = 'ffffffffffffB'
        self.raw_fmt = 'HHHHHHHHHhhhHHHHHH 13B'
        self.cchans = ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'accel3_x', 'accel3_y', 'accel3_z', 'checkbyte']
        self.rchans= ['haccel_x', 'haccel_y', 'haccel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'laccel_x', 'laccel_y', 'laccel_z', 'tempaccel_x', 'tempaccel_y', 'tempaccel_z', 'tempgyro_x', 'tempgyro_y', 'tempgyro_z', 'zeropadding', 'rawchecksum']
        self.date_stamp = datetime.datetime.today().strftime("%Y%m%dT%H%M%S")

    def acquire_id(self):
        self.serial.write('\x29')
        r = self.serial.read(49)

        try:
            rdata = struct.unpack('HHHHHHHHHhhhHHHHHH 2BBBQB', r)
            if rdata[21]<16:
                t = '0'+hex(rdata[21])[2:3]
            else:
                t = hex(rdata[21])[2:4]

            if rdata[20]<16:
                q = '0'+hex(rdata[20])[2:3]
            else:
                q = hex(rdata[20])[2:4]

            self.ID_name = hex(rdata[22])[0:14] + t + q
        except:
            self.ID_name = "<Unknown>"
        finally:
            return self.ID_name

    def acquire_serial(self):
        return raw_input("Input IMU serial number: ")

    def acquire(self):
        self.serial.write('\xe3')
        cal_msg = self.serial.read(49)
        self.serial.write('\x29')
        raw_msg = self.serial.read(49)

        cal_data = struct.unpack(self.cal_fmt, cal_msg)
        raw_data = struct.unpack(self.raw_fmt, raw_msg)
        tdata = tuple([time.clock()])

        return list(tdata + cal_data + raw_data)

    def acquire_log(self):
        print "Acquiring data, press Ctrl-C to stop",
        data = []
        x = 0
        try:
            while True:
                x = x + 1
                sample = self.acquire()
                data.append(sample)
                if (x % 100 == 0):
                    sys.stdout.write('.')
                    sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        print
        return data

#imu_interface = IMUInterface(9) #("/dev/ttyUSB0")
#imu_interface.acquire_id()
#imu_interface.acquire_log()
