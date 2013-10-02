#Copyright 2013 Google Inc. All Rights Reserved.
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

import serial, struct

fmt = 'ffffffffffffB'
chans = ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z', 'accel3_x', 'accel3_y', 'accel3_z', 'checkbyte']

ser = serial.Serial()
ser.port = 4
ser.setBaudrate(937500)
ser.timeout = 1

ser.open()
ser.write('\xe3')
s = ser.read(49)
try:
   data = struct.unpack(fmt, s)
   for res in zip(chans, data):
      print res[0], res[1]
except:
   print "communication error..."

ser.close()
