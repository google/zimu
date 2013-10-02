%Copyright 2013 Google Inc. All Rights Reserved.
%
%Licensed under the Apache License, Version 2.0 (the "License");
%you may not use this file except in compliance with the License.
%You may obtain a copy of the License at
%
%    http://www.apache.org/licenses/LICENSE-2.0
%
%Unless required by applicable law or agreed to in writing, software
%distributed under the License is distributed on an "AS IS" BASIS,
%WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%See the License for the specific language governing permissions and
%limitations under the License.

% Plot raw IMU data

data = calIMU10_20120611T1637;
close all;

t = data(:,1);
accel_conv = data(:,2:4);
gyro_conv = data(:,5:7);
mag_conv = data(:,8:10);
lowg_conv = data(:,11:13);

accel_raw = data(:,15:17);
gyro_raw = data(:,18:20);
mag_raw = data(:,21:23);
lowg_raw = data(:,24:26);

% Plot converted IMU data
clear ax;
figure('name', 'IMU converted', 'WindowStyle', 'dock');
ax(1) = subplot(4,1,1); plot(t, accel_conv); title('Accel');
ax(2) = subplot(4,1,2); plot(t, gyro_conv); title('Gyro');
ax(3) = subplot(4,1,3); plot(t, mag_conv); title('Mag');
ax(4) = subplot(4,1,4); plot(t, lowg_conv); title('Low Accel');
linkaxes(ax,'x');

% Plot raw IMU data
clear ax;
figure('name', 'IMU raw', 'WindowStyle', 'dock');
ax(1) = subplot(4,1,1); plot(t, accel_raw); title('Accel');
ax(2) = subplot(4,1,2); plot(t, gyro_raw); title('Gyro');
ax(3) = subplot(4,1,3); plot(t, mag_raw); title('Mag');
ax(4) = subplot(4,1,4); plot(t, lowg_raw); title('Low Accel');
linkaxes(ax,'x');

% Plot Norms
clear ax;
figure('name', 'IMU norms', 'WindowStyle', 'dock');
ax(1) = subplot(4,1,1); plot(t, vnorms(accel_conv)); title('Accel');
ax(2) = subplot(4,1,2); plot(t, vnorms(gyro_conv)); title('Gyro');
ax(3) = subplot(4,1,3); plot(t, vnorms(mag_conv)); title('Mag');
ax(4) = subplot(4,1,4); plot(t, vnorms(lowg_conv)); title('Low Accel');
linkaxes(ax,'x');
