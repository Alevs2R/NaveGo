% navego_example_real_ekinox: post-processing integration of both Ekinox 
% IMU and Ekinox GNSS data.
%
% Main goal: to integrate IMU and GNSS measurements from Ekinox-D sensor 
% which includes both IMU and GNSS sensors.
%
% Sensors dataset was generated driving a vehicle through the streets of 
% Turin city (Italy).
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
% Reference:
%
%   SBG Systems. SBG Ekinox-D High Accuracy Inertial System Brochure, 
% Tactical grade MEMS Inertial Systems, v1.0. February 2014. 
%
%   R. Gonzalez and P. Dabove. Performance Assessment of an Ultra Low-Cost 
% Inertial Measurement Unit for Ground Vehicle Navigation. Sensors 2019,  
% 19(18). https://www.mdpi.com/530156.
%
% Version: 002
% Date:    2020/10/22
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

% NOTE: NaveGo assumes that IMU is aligned with respect to body-frame as 
% X-forward, Y-right, and Z-down.

clc
close all
clear
matlabrc

addpath ../../.
addpath ../../simulation/
addpath ../../conversions/
addpath ../../performance_analysis/

versionstr = 'NaveGo, release v1.2';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting real INS/GNSS integration... \n')

%% PARAMETERS

% Comment any of the following parameters in order to NOT execute a 
% particular portion of code

INS_GNSS = 'ON';
PLOT     = 'ON';

if (~exist('INS_GNSS','var')), INS_GNSS = 'OFF'; end
if (~exist('PLOT','var')),     PLOT     = 'OFF'; end

%% CONVERSION CONSTANTS

G =  9.81508;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% GKV IMU 

fprintf('NaveGo: loading GKV IMU data... \n')

load orient_imu

%% ORIENT GNSS 

fprintf('NaveGo: loading Orient GNSS data... \n')

load orient_gnss

% ekinox_gnss.eps = mean(diff(ekinox_imu.t)) / 2; %  A rule of thumb for choosing eps.

%% INS/GNSS integration

if strcmp(INS_GNSS, 'ON')
    
    fprintf('NaveGo: INS/GNSS integration... \n')
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    nav_orient = ins_gnss_stdm_vector(orient_imu, orient_gnss, 'quaternion'); %
    % ---------------------------------------------------------------------
    
    save nav_orient.mat nav_orient
    
else
    
    load nav_orient
end

%% Print navigation time

to = (nav_orient.t(end) - nav_orient.t(1));

fprintf('NaveGo: navigation time under analysis is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% Interpolation of GNSS

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

% [gnss_r, ref_g] = navego_interpolation (orient_gnss, nav_orient);

%% PLOT

if (strcmp(PLOT,'ON'))
    
   orient_plot (orient_gnss, nav_orient)
end

% %% PLOT LATITUDE
% 
% figure;
% plot(orient_gnss.t, orient_gnss.lat.*R2D);
% xlabel('Time [s]')
% ylabel('[deg]')
% legend('GNSS');
% title('LATITUDE');
% grid
% 
% %% PLOT accelerations
% figure;
% subplot(311)
% plot(orient_imu.t, orient_imu.fb(:,1));
% xlabel('Time [s]')
% ylabel('m/s^2')
% legend('GKV');
% title('acceleration x');
% grid
% 
% subplot(312)
% plot(orient_imu.t, orient_imu.fb(:,2));
% xlabel('Time [s]')
% ylabel('m/s^2')
% legend('GKV');
% title('acceleration y');
% grid
% 
% subplot(313)
% plot(orient_imu.t, orient_imu.fb(:,3));
% xlabel('Time [s]')
% ylabel('m/s^2')
% legend('GKV');
% title('acceleration z');
% grid