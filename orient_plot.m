function  orient_plot (gnss, gnss_heading, nav_e, gkv_gnss, gnss_outrage, idx)
% navego_plot: plots results from INS/GNSS integration dataset.
%
% INPUT:
%   gnss,         GNSS dataset.
%   gnss_heading, heading and roll angle from 2 antennas (GNSS) 
%   nav_e,        INS/GNSS integration dataset.
%   gkv_gnss,     reference trajectory from gkv INS/GNSS solution
%
% OUTPUT
%   several figures.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% Version: 009
% Date:    2020/06/29
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

% Standard deviation * 3 vector from navigation estimates
sig3_v = abs(nav_e.Pp(:, 1:16:end).^(0.5)) .* 3; % Only take diagonal elements from Pp
sig3_priori = abs(nav_e.Pi(:, 1:16:end).^(0.5)) .* 3; % Only take diagonal elements from Pi

[nav_e_x_utm,nav_e_y_utm,~] = deg2utm(nav_e.lat.*R2D,nav_e.lon.*R2D);
[gnss_x_utm,gnss_y_utm,~] = deg2utm(gnss.lat.*R2D,gnss.lon.*R2D);

offset_x = nav_e_x_utm(1);
offset_y = nav_e_y_utm(1);
nav_e_x_utm = nav_e_x_utm - offset_x;
nav_e_y_utm = nav_e_y_utm - offset_y;
gnss_x_utm = gnss_x_utm - offset_x;
gnss_y_utm = gnss_y_utm - offset_y;
gkv_x_utm = gkv_gnss.utm_x - offset_x;
gkv_y_utm = gkv_gnss.utm_y - offset_y;

% TRAJECTORY
% figure;
% plot3(nav_e.lon.*R2D, nav_e.lat.*R2D, nav_e.h, '-ob')
% hold on
% plot3(gnss.lon.*R2D, gnss.lat.*R2D, gnss.h)
% plot3(nav_e.lon(1).*R2D, nav_e.lat(1).*R2D, nav_e.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
% axis tight
% title('TRAJECTORY')
% xlabel('Longitude [deg]')
% ylabel('Latitude [deg]')
% zlabel('Altitude [m]')
% view(0, 90)
% legend('INS/GNSS', 'GNSS', 'Starting point', 'Location', 'best');
% grid

figure;
plot3(nav_e_x_utm, nav_e_y_utm, nav_e.h)
hold on
plot3(gkv_x_utm, gkv_y_utm, gkv_gnss.h, '-.')
scatter3(gnss_x_utm, gnss_y_utm, gnss.h, 1, 'k')
plot3(nav_e_x_utm(1), nav_e_y_utm(1), nav_e.h(1), 'ob', 'MarkerSize', 10, 'LineWidth', 2)
if (strcmp(gnss_outrage, 'ON'))
    plot3(gnss_x_utm(idx), gnss_y_utm(idx), gnss.h(idx), 'or', 'MarkerSize', 10, 'LineWidth', 2)
end
axis tight
title('TRAJECTORY')
xlabel('UTM X [m]')
ylabel('UTM Y [m]')
zlabel('Altitude [m]')
view(0, 90)
if (strcmp(gnss_outrage, 'ON'))
    legend('NaveGo INS/GNSS', 'GKV INS/GNSS', 'GNSS', 'Starting point', 'Disable GNSS point', 'Location', 'best');
else
    legend('NaveGo INS/GNSS', 'GKV INS/GNSS', 'GNSS', 'Starting point', 'Location', 'best');
end    
axis equal;
grid

% ax = gca;
% ax.XRuler.Exponent = 0;

% ATTITUDE
figure;
subplot(311)
plot(nav_e.t, R2D.*nav_e.roll,'-.b',gnss_heading.t, R2D.*gnss_heading.roll,'-c');
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS', 'GNSS roll angle [2 antenna]');
title('ROLL');
ylim([-5,5]);
grid

subplot(312)
plot(nav_e.t, R2D.*nav_e.pitch,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS')
title('PITCH');
grid

subplot(313)
plot(nav_e.t, R2D.*nav_e.yaw,'-b');
ylabel('[deg]')
xlabel('Time [s]')
legend('INS/GNSS')
title('YAW');
grid

% VELOCITIES
figure;
subplot(311)
plot(gnss.t, gnss.vel(:,1),'-c', nav_e.t, nav_e.vel(:,1),'-.b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS');
title('NORTH VELOCITY');
grid

subplot(312)
plot(gnss.t, gnss.vel(:,2),'-c', nav_e.t, nav_e.vel(:,2),'-.b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS');
title('EAST VELOCITY');
grid

subplot(313)
plot(gnss.t, gnss.vel(:,3),'-c', nav_e.t, nav_e.vel(:,3),'-.b' );
xlabel('Time [s]')
ylabel('[m/s]')
legend('GNSS', 'INS/GNSS');
title('DOWN VELOCITY');
grid


% POSITION
figure;
subplot(311)
plot(gnss.t, gnss.lat.*R2D, '-c', nav_e.t, nav_e.lat.*R2D, '-.b');
xlabel('Time [s]')
ylabel('[deg]')
legend('GNSS', 'INS/GNSS' );
title('LATITUDE');
grid

subplot(312)
plot(gnss.t, gnss.lon.*R2D, '-c', nav_e.t, nav_e.lon.*R2D, '-.b');
xlabel('Time [s]')
ylabel('[deg]')
legend('GNSS', 'INS/GNSS' );
title('LONGITUDE');
grid

subplot(313)
plot(gnss.t, gnss.h, '-c', nav_e.t, nav_e.h, '-.b')
xlabel('Time [s]')
ylabel('[m]')
legend('GNSS', 'INS/GNSS');
title('ALTITUDE');
grid

% BIAS ESTIMATION
figure;
subplot(311)
plot(nav_e.tg, nav_e.b(:, 1).*R2D, '-.b');
xlabel('Time [s]')
ylabel('[deg]')
title('KF BIAS GYRO X ESTIMATION');
grid

subplot(312)
plot(nav_e.tg, nav_e.b(:, 2).*R2D, '-.b');
xlabel('Time [s]')
ylabel('[deg]')
title('KF BIAS GYRO Y ESTIMATION');
grid

subplot(313)
plot(nav_e.tg, nav_e.b(:, 3).*R2D, '-.b');
xlabel('Time [s]')
ylabel('[deg]')
title('KF BIAS GYRO Z ESTIMATION');
grid

figure;
subplot(311)
plot(nav_e.tg, nav_e.b(:, 4), '-.b');
xlabel('Time [s]')
ylabel('[m/s^2]')
title('KF BIAS ACCR X ESTIMATION');
grid

subplot(312)
plot(nav_e.tg, nav_e.b(:, 5), '-.b');
xlabel('Time [s]')
ylabel('[m/s^2]')
title('KF BIAS ACCR Y ESTIMATION');
grid

subplot(313)
plot(nav_e.tg, nav_e.b(:, 6), '-.b');
xlabel('Time [s]')
ylabel('[m/s^2]')
title('KF BIAS ACCR Z ESTIMATION');
grid

figure;
subplot(311)
plot(nav_e.tg, nav_e.K(:, 49));
xlabel('Time [s]')
ylabel('Gain')
title('Kalman gain for latitude');
subplot(312)
plot(nav_e.tg, nav_e.K(:, 65));
xlabel('Time [s]')
ylabel('Gain')
title('Kalman gain for longitude');
subplot(313)
plot(nav_e.tg, nav_e.K(:, 81));
xlabel('Time [s]')
ylabel('Gain')
title('Kalman gain for height');
grid

% variance
figure;

R = 6.3781 * 10^6;              % Earth's radius in m
std_lat = [ 0; sig3_v(2:end,7) ];
std_lon = [ 0; sig3_v(2:end,8) ];
a = sin( std_lat ./ 2 ).^2 + cos( gnss.lat).* cos( gnss.lon) .* ...
        sin( std_lon ./ 2 ).^2;
c = 2 .* atan2 ( sqrt(a), sqrt (1-a) );
std_hor_posteriori = R .* c;

std_lat = [ 0; sig3_priori(2:end,7) ];
std_lon = [ 0; sig3_priori(2:end,8) ];
a = sin( std_lat ./ 2 ).^2 + cos( gnss.lat).* cos( gnss.lon) .* ...
        sin( std_lon ./ 2 ).^2;
c = 2 .* atan2 ( sqrt(a), sqrt (1-a) );
std_hor_priori = R .* c;

plot (nav_e.tg, std_hor_priori);
hold on;
plot (nav_e.tg, std_hor_posteriori);
title("Standart deviation of horizontal position")
legend("from priopi covariance matrix", "from posteriori covariance matrix")
ylabel("meters")
xlabel("Time [s]")

% Free accelerations

figure;
title("ACCELERATIONS WITH BIASES, GRAVITY AND CORIOLIS FORCES SUBSTRACTED");
subplot(311)
plot(nav_e.t, nav_e.fb_free(:, 1));
xlabel('Time [s]')
ylabel('[m/s^2]')
title('FREE ACCELERATION X');
ylim([-3,3]);
grid

subplot(312)
plot(nav_e.t, nav_e.fb_free(:, 2));
xlabel('Time [s]')
ylabel('[m/s^2]')
title('FREE ACCELERATION Y');
ylim([-3,3]);
grid

subplot(313)
plot(nav_e.t, nav_e.fb_free(:, 3));
xlabel('Time [s]')
ylabel('[m/s^2]')
title('FREE ACCELERATION Z');
ylim([-3,3]);
grid

