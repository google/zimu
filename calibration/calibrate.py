#!/usr/bin/env python

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

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages
import csv
import scipy.optimize
import scipy.signal
import math
import sys
import argparse
import datetime
import socket

# For reading IMU data over serial
import imu_interface

# Generic sensor class for fitting calibration to data
class Sensor:
    def __init__(self, name, prefix, nominal_scale, units, signs, nominal_bias = 0, reference_value = 1, resample_count = 1500, window_size = 250, noise_threshold = 15):
        self.name = name
        self.prefix = prefix
        self.nominal_bias = numpy.array([nominal_bias] * 3)
        self.signs = numpy.array(signs)
        self.nominal_scale = nominal_scale
        self.reference_value = reference_value
        self.units = units
        self.resample_count = resample_count
        self.window_size = window_size
        self.noise_threshold = noise_threshold
        self.raw_data = None
        self.raw_norms = None
        self.filtered_data = None
        self.filtered_norms = None
        self.filtered_idx = None
        self.resampled_data = None
        self.resampled_norms = None
        self.calibrated_data = None
        self.calibrated_norms = None
        self.scale = [1, 1, 1]
        self.bias = self.nominal_bias
        self.temp_n0 = [0, 0, 0]
        self.temp_sens = [1, 1, 1]
        self.plot_callback = lambda : None

    def set_plot_callback(self, cb):
        self.plot_callback = cb

    def set_header_file(self, out):
        self.header_out = out

    def remove_bias(self):
        self.raw_data[:,:] = self.raw_data[:,:] - self.nominal_bias

    def add_bias(self):
        self.resampled_data[:,:] = self.resampled_data[:,:] + self.nominal_bias

    def raw_plot(self):
        uncalibrated_figure = plt.figure()
        ax = uncalibrated_figure.add_subplot(111, projection='3d')
        ax.set_title("Uncalibrated " + self.name)
        ax.scatter(self.raw_data[::30,0], self.raw_data[::30,1], self.raw_data[::30,2], marker='^')
        self.plot_callback()

        self.xy = plt.figure()
        self.xy.subplots_adjust(hspace=0.5)
        s1 = self.xy.add_subplot(311)
        s1.plot(self.raw_data[::30,0], c='r')
        s1.plot(self.raw_data[::30,1], c='g')
        s1.plot(self.raw_data[::30,2], c='b')
        s1.set_xlabel('Samples')

    def uncalibrated_norm_plot(self):
        ax3 = self.xy.add_subplot(312)
        ax3.plot(self.raw_norms)
        ax3.scatter(self.filtered_idx[::30], self.filtered_norms[::30], c='r', marker='^')
        ax3.set_title("Uncalibrated norm " + self.name)
        ax3.set_xlabel('Samples')
        ax3.set_ylabel(self.units)

    def calibrated_norm_plot(self):
        ss2 = self.xy.add_subplot(313)
        ss2.plot(self.calibrated_norms[::15])
        ss2.set_xlabel('Samples')
        ss2.set_ylabel(self.units)
        ss2.set_title('Calibrated Norm ' + self.name)
        self.plot_callback()

    def calibrated_plot(self):
        calibrated_figure = plt.figure()
        ss1 = calibrated_figure.add_subplot(111, projection='3d')
        ss1.scatter(self.calibrated_data[::10,0], self.calibrated_data[::10,1], self.calibrated_data[::10,2])
        ss1.set_title('Calibrated ' + self.name)

    def plot_temp(self):
        temp_plt = plt.figure()
        tp = temp_plt.add_subplot(111)
        tp.set_title("Temps for " + self.name + " (X,Y,Z)")
        tp.set_xlabel('Samples')
        tp.set_ylabel('Temp ADC ' + self.name)
        for idx, axis, color in zip(range(0,3), ['X', 'Y', 'Z'], ['r','g','b']):
            tp.plot(self.temp_x[::40,idx], c=color)
        self.plot_callback()
        splot = plt.figure()
        splot.subplots_adjust(hspace=0.5)
        for idx, axis in zip(range(0,3), ['X', 'Y', 'Z']):
            s = splot.add_subplot(311 + idx)
            s.set_title('Temp ' + axis + ' vs ' + self.name + ' ' + axis)
            s.scatter(self.temp_x[::20,idx], self.temp_y[::20,idx], marker='^')
            s.set_ylabel(self.name + ' ' + axis)
        self.plot_callback()

    def print_data(self, label, data, norms):
        print label, data.shape[0],  "samples, std(norm):",  numpy.std(norms), "guess norm", numpy.mean(norms), self.units
        self.write_comment(label + " " + str(data.shape[0]) + " samples, std(norm): " + str(numpy.std(norms)) + " guess norm " + str(numpy.mean(norms)) + " " + self.units)

    def write_comment(self, s):
        self.header_out.write("// " + s + "\n")

    def write_define(self, name, value):
        self.header_out.write("#define " + name + " ")
        self.header_out.write(str(value))
        self.header_out.write("\n")

    def write_calibration(self):
        for idx, axis in zip(range(0,3), ['X', 'Y', 'Z']):
            self.write_comment(self.name + " " + axis)
            p = self.prefix + axis
            self.write_define(p + "_CGF", self.scale[idx] * self.signs[idx])
            self.write_define(p + "_CBO", str((self.bias[idx] - self.nominal_bias[idx]) * self.scale[idx] * self.nominal_scale))
            self.write_define(p + "_CTN", str(self.temp_n0[idx]))
            self.write_define(p + "_CTS", str(self.temp_sens[idx] * self.signs[idx] * self.nominal_scale))
            self.header_out.write("\n")

    def find_norms(self, data):
        norms = []
        for row in data:
            norms.append(numpy.linalg.norm(row) * self.nominal_scale)
        return norms

    def fill_data(self):
        self.remove_bias()
        self.raw_norms = self.find_norms(self.raw_data)

    def filter_data(self):
        filtered_meas = []
        self.filtered_idx = []
        guess_mean = numpy.mean(self.raw_norms)
        for i in range(self.window_size, len(self.raw_data)-self.window_size):
            noise = self.raw_data[i-self.window_size:i+self.window_size,:].std(axis=0)
            if self.raw_norms[i] > guess_mean * 0.5:
                if numpy.linalg.norm(noise) < self.noise_threshold:
                    filtered_meas.append(self.raw_data[i,:])
                    self.filtered_idx.append(i)
        self.filtered_data = numpy.array(filtered_meas)
        self.filtered_norms = self.find_norms(self.filtered_data)

    def downsample_data(self):
        if self.filtered_data.shape[0] < self.resample_count:
            print "Not enough points"
            raise Exception
        self.resampled_data = scipy.signal.resample(self.filtered_data, self.resample_count)
        self.resampled_norms = self.find_norms(self.resampled_data)

    def calibrate_data(self):
        self.calibrated_data = (self.resampled_data - self.bias) * self.scale
        self.calibrated_norms = self.find_norms(self.calibrated_data)

    def fit_temp(self):
        print "Fitting Temp for", self.name
        self.x0 = [1, 0]
        self.temp_n0 = [0, 0, 0]
        self.temp_sens = [1, 1, 1]
        zero = 0
        for idx in range(0, 3):
            self.temp_n0[idx] = numpy.mean(self.temp_x[:,idx])
            temp_mean = numpy.mean(self.temp_y[:,idx])
            (self.temp_sens[idx], zero) = numpy.polyfit(self.temp_x[::3,idx] - self.temp_n0[idx], self.temp_y[::3,idx] - temp_mean, 1)
        print

    def fit_calibration(self):
        print "Fitting",
        self.x0 = self.signs.tolist() + self.nominal_bias.tolist()
        self.counter = 0
        self.x0 = scipy.optimize.fmin(self.error_function, self.x0, maxiter=5000, maxfun=5000, callback=self.update)
        self.bias = self.x0[3:6]
        self.scale = self.x0[0:3]
        print

    def error_function(self, x0):
        bias = x0[3:6]
        A = numpy.diag(x0[0:3])
        sum = 0
        r = self.reference_value / self.nominal_scale
        for row in self.resampled_data:
            row = row - bias
            row = row.T * A
            norm_error = numpy.linalg.norm(row) - r
            sum = sum + norm_error ** 2
        return sum

    def update(self, x0):
        if self.counter % 3 == 0:
            sys.stdout.write(".")
            sys.stdout.flush()
        self.counter = self.counter + 1

    def run_calibration(self, data):
        self.fill_data(data)
        self.raw_plot()
        self.print_data("Logged:", self.raw_data, self.raw_norms)

        self.filter_data()
        self.print_data("Filtered:", self.filtered_data, self.filtered_norms)

        self.downsample_data()
        self.print_data("Resampled:", self.resampled_data, self.resampled_norms)
        self.uncalibrated_norm_plot()

        self.add_bias()
        self.fit_calibration()
        self.calibrate_data()
        self.calibrated_plot()
        self.calibrated_norm_plot()

    def run_temp(self, data):
        self.fill_temp_data(data)
        self.fit_temp()
        self.plot_temp()

class Gyro(Sensor):
    def __init__(self):
        Sensor.__init__(self, "gyro", prefix="G", nominal_bias=2048, nominal_scale=1285.8/4096.0, reference_value=270.0, units="deg/sec", signs=[1, 1, -1])

    def fill_data(self, data):
        self.raw_data = data[:,17:20]
        Sensor.fill_data(self)

    def fill_temp_data(self, data):
        self.temp_x = data[:,29:32]
        self.temp_y = data[:,17:20]

class GyroNoScale(Gyro):
    def error_function(self, x0):
        bias = x0[0:3]
        sum = 0
        for row in self.resampled_data:
            row = row - bias
            row = row * self.signs
            norm_error = numpy.linalg.norm(row)
            sum = sum + norm_error ** 2
        return sum

    def fit_calibration(self):
        print "Fitting",
        self.counter = 0
        self.x0 = self.nominal_bias.tolist()
        self.x0 = scipy.optimize.fmin(self.error_function, self.x0, maxiter=5000, maxfun=5000, callback=self.update)
        self.bias = self.x0[0:3]
        self.scale = numpy.array(self.signs)
        print

class Mag(Sensor):
    def __init__(self):
        Sensor.__init__(self, "mag", prefix="M", nominal_bias=2048, nominal_scale=4.0/4096.0, reference_value=0.490386, units="Gauss", window_size=50, noise_threshold=500, signs=[-1, 1, 1])
    def fill_data(self, data):
        self.raw_data = data[:,20:23]
        Sensor.fill_data(self)
    def fill_temp_data(self, data):
        self.temp_x = data[:,29:32]
        self.temp_y = data[:,20:23]

class Accel(Sensor):
    def __init__(self):
        Sensor.__init__(self, "accel", prefix="A", nominal_bias=2048, nominal_scale=125.0/4096.0, reference_value=1.0, units="g", window_size=200, noise_threshold=30, signs=[-1, -1, 1])
    def fill_data(self, data):
        self.raw_data = data[:,14:17]
        Sensor.fill_data(self)
    def fill_temp_data(self, data):
        self.temp_x = data[:,26:29]
        self.temp_y = data[:,14:17]

class AccelLow(Sensor):
    def __init__(self):
        Sensor.__init__(self, "accel low", prefix="A3", nominal_scale=12.0/4096.0, units="g", noise_threshold=250, signs=[1, 1, 1])
    def fill_data(self, data):
        self.raw_data = data[:,23:26]
        Sensor.fill_data(self)
    def fill_temp_data(self, data):
        self.temp_x = data[:,29:32]
        self.temp_y = data[:,23:26]
    def fit_temp(self):
        self.temp_n0 = [0, 0, 0]
        self.temp_sens = [0, 0, 0]

class IMUCalibration():
    def __init__(self, port, mag_log, accel_log, gyro_log, temp_log):
        self.imu_id = "Unknown"
        self.imu_serial = "Unknown"
        try:
            self.interface = imu_interface.IMUInterface(port)
            self.imu_id = self.interface.acquire_id()
            self.imu_serial = self.interface.acquire_serial()
        except:
            pass
        self.result_name = "IMU" + self.imu_serial + '_' + datetime.datetime.today().strftime("%Y%m%dT%H%M")
        self.result_headername = self.result_name + '.h'
        self.result_pdfname = self.result_name + '.pdf'
        self.result_csv = self.result_name + '.csv'
        self.pdf = PdfPages(self.result_pdfname)
        self.mag_log = mag_log
        self.accel_log = accel_log
        self.gyro_log = gyro_log
        self.temp_log = temp_log
        self.mag = Mag()
        self.mag.set_plot_callback(self.save_plot)
        self.accel_l = AccelLow()
        self.accel_l.set_plot_callback(self.save_plot)
        self.accel = Accel()
        self.accel.set_plot_callback(self.save_plot)
        self.gyro = Gyro()
        self.gyro.set_plot_callback(self.save_plot)

    def save_plot(self):
        self.pdf.savefig()

    def get_data(self, sensor_name, log_name):
        if log_name:
            print "Loading", log_name, "for", sensor_name
            self.gyro.write_comment("Using log: " + log_name + " for " + sensor_name)
            return numpy.genfromtxt(log_name, delimiter=',')
        else:
            print "Logging data for", sensor_name
            return numpy.array(self.interface.acquire_log())

    def make_log(self, log_name):
        log_data = self.get_data(log_name + " file", None)
        numpy.savetxt(log_name + self.result_csv, log_data, fmt="%-.9G", delimiter=',')

    def run(self):
        output = open(self.result_headername, 'w')
        self.gyro.set_header_file(output)
        self.accel.set_header_file(output)
        self.accel_l.set_header_file(output)
        self.mag.set_header_file(output)

        self.gyro.write_comment("IMU calibration generated with %s on %s at %s" % (sys.argv[0], socket.gethostname(), str(datetime.datetime.today())))
        self.gyro.write_comment("ID: %s" % self.imu_serial)
        self.gyro.write_comment("SID: %s" % self.imu_id)

        try:
            gyro_data = self.get_data("gyro", self.gyro_log)
            if self.gyro_log:
                self.gyro.run_calibration(gyro_data)
            else:
                self.gyro = GyroNoScale()
                self.gyro.set_header_file(output)
                self.gyro.run_calibration(gyro_data)
        except Exception as e:
                print "Gyro data unavailable, skipping: ", e

        try:
            accel_data = self.get_data("accel", self.accel_log)
            self.accel.run_calibration(accel_data)
            self.accel_l.run_calibration(accel_data)
        except Exception as e:
            print "Accel data unavailable, skipping", e

        try:
            mag_data = self.get_data("mag", self.mag_log)
            self.mag.run_calibration(mag_data)
        except Exception as e:
            print "Mag data unavailable, skipping", e

        try:
            temp_data = self.get_data("temperature", self.temp_log)
            self.mag.run_temp(temp_data)
            self.accel.run_temp(temp_data)
            self.accel_l.run_temp(temp_data)
            self.gyro.run_temp(temp_data)
        except Exception as e:
            print "Temperature data unavailable, skipping", e

        self.gyro.write_calibration()
        self.accel.write_calibration()
        self.mag.write_calibration()
        self.accel_l.write_calibration()

        output.close()
        self.pdf.close()
        plt.show()

def get_options():
    parser = argparse.ArgumentParser(description='IMU calibration utility')
    parser.add_argument('-p', '--port', default=9, help='serial port for IMU') #/dev/ttyUSB0
    parser.add_argument('-l', '--log', help='output csv log file for logging only - optional')
    parser.add_argument('-m', '--mag', help='input csv log file for mag calibration (optional, otherwise acquire')
    parser.add_argument('-a', '--accel', help='input csv log file for accel calibration (optional, otherwise acquire')
    parser.add_argument('-t', '--temp',  help='input csv log file for temp calibration')
    parser.add_argument('-g', '--gyro', help='input csv log file for gyro calibration (optional, turntable version, otherwise acquire static version without scaling)')
    return parser.parse_args()

def main():
    args = get_options()
    calibration = IMUCalibration(args.port, args.mag, args.accel, args.gyro, args.temp)
    if (args.log):
        calibration.make_log(args.log)
    else:
        calibration.run()

if __name__ == '__main__':
    main()
