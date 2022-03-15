close all;
clear;
clc;

global reward counter flag TP old_pos;
reward = [];
counter = 1;
flag = 0;
TP = 0;

mdl = "vehicle_control";
load_system(mdl)

%% load the scene data file generated from Driving Scenario Designer
load('curveLowVel.mat');

%% define reference points
refPose = data.ActorSpecifications.Waypoints;
%refPose = [linspace(0,150,15);zeros(1,15)]';
xRef = refPose(:,1);
yRef = -refPose(:,2);
X_o = refPose(1,1)-10; % initial vehicle x position
Y_o = -refPose(1,2)+10; % initial vehicle y position
psi_o = 0; % initial yaw angle
old_pos = [X_o,Y_o];

%% define reference time for plotting 
s = size(xRef);
Ts = 30; % simulation time
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points. 

%% define parameters used in the models
%rng(1);

%vel = 9; %m/s
L = 3; % bicycle length
%ld = 1; % lookahead distance

vel = randi([1,10],1,1);
%ld = randi([0,10],1,1);
ld = vel/2;

%% Run simulation
out = sim(mdl);
scatter(yRef,xRef,'green','filled')
%calculate_error(xRef,yRef,tRef);
