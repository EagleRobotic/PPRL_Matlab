close all; clear; clc;

mdl = 'stanley_model';
addpath(genpath('Images'));
load_system(mdl)
sampling = 0.01;

%% load the scene data file generated from Driving Scenario Designer
load('data/curveLowVel.mat');
%load('data/USCity.mat');
%refPose = [linspace(0,150,15);zeros(1,15)]';

%% define reference points
refPose = data.ActorSpecifications.Waypoints;
xRef = refPose(:,1);
yRef = -refPose(:,2);

%% data preprocess
% calculate distance vector
distancematrix = squareform(pdist(refPose));
distancesteps = zeros(length(refPose)-1,1);
for i = 2:length(refPose)
    distancesteps(i-1,1) = distancematrix(i,i-1);
end
totalDistance = sum(distancesteps); % Total distance travelled
distbp = cumsum([0; distancesteps]); % Distance for each waypoint
gradbp = linspace(0,totalDistance,50); % Linearize distance

% linearize X and Y vectors based on distance
xRef2 = interp1(distbp,xRef,gradbp);
yRef2 = interp1(distbp,yRef,gradbp);
yRef2s = smooth(gradbp,yRef2); % smooth waypoints
xRef2s = smooth(gradbp,xRef2); % smooth waypoints

thetaRef = zeros(length(gradbp),1);
for i = 2:length(gradbp)
    thetaRef(i,1) = atan2d((yRef2(i)-yRef2(i-1)),(xRef2(i)-xRef2(i-1)));
end
thetaRefs = smooth(gradbp,thetaRef); % smooth of theta
psi_o = thetaRefs(1)*(pi/180); % initial yaw angle

X_o = xRef(1); % initial vehicle x position
Y_o = yRef(1); % initial vehicle y position

direction = ones(length(gradbp),1);
curvature = getCurvature(xRef2,yRef2);

%% define reference time for plotting 
s = size(xRef);
Ts = 50; % simulation time
data.StopTime = Ts;
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points. 

%% define parameters used in the models
%max vel 15
vel = 5; %m/s 
L = 3; % bicycle length
ld = 5; %Look-ahead distance

sim(mdl);
scatter(yRef,xRef,'green','filled')


%% Curvature Function
function curvature = getCurvature(xRef,yRef)
% Calculate gradient by the gradient of the X and Y vectors
DX = gradient(xRef);
D2X = gradient(DX);
DY = gradient(yRef);
D2Y = gradient(DY);
curvature = (DX.*D2Y - DY.*D2X) ./(DX.^2+DY.^2).^(3/2);
end