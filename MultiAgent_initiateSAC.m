close all; clear; clc;

%mdl = 'new_model';
mdl = 'MultiAgentModel';
load_system(mdl)
sampling = 0.1;

%% load the scene data file generated from Driving Scenario Designer
load('data/curveLowVel.mat');
%load('data/USCity.mat');
%refPose = [linspace(0,150,15);zeros(1,15)]';

%% define reference points
%refPose = data.ActorSpecifications(1,67).Waypoints;
refPose = data.ActorSpecifications.Waypoints;
xRef = refPose(:,1);
yRef = -refPose(:,2);

X_o = refPose(1,1)+0; % initial vehicle x position
Y_o = -refPose(1,2)+20; % initial vehicle y position
psi_o = 0; % initial yaw angle

dx = abs(X_o - xRef);  % distance to all waypoints at the current time
dy = abs(Y_o - yRef);
lp = hypot(dx, dy); %Find hypotenuse 
[sWayPoints, idx] = sort(lp); % sort the way points according to distance
ni = idx(1); % nearest point candidate
o1x = xRef(idx(1));
o1y = yRef(idx(1));

o2x = xRef(idx(2));
o2y = yRef(idx(2));

cnormal_xRef = linspace(o1x, o2x, 10);
cnormal_yRef = linspace(o1y, o2y, 10);

cdx = abs(X_o - cnormal_xRef);
cdy = abs(Y_o - cnormal_yRef);

lpc = hypot(cdx, cdy); %Find hypotenuse 
[sWayPointsc, idxc] = sort(lpc); % sort the way points according to distance
nic = idxc(1); % nearest point candidate

xteInt = sqrt((cnormal_xRef(idxc(1))-X_o)^2+(cnormal_yRef(idxc(1))-Y_o)^2);

%% define reference time for plotting 
s = size(xRef);
Ts = 100; % simulation time
data.StopTime = Ts;
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points. 

%% define parameters used in the models
%max vel 15
vel = 5; %m/s 
L = 3; % bicycle length
ld = 5; % lookahead distance

%% RL Spesifications

obsInfo_1 = rlNumericSpec([2 1]);
obsInfo_1.Name = 'observations';
obsInfo_1.Description = 'Integrated CTE and CTE';

actInfo_1 = rlNumericSpec([1 1],'LowerLimit',0.5,'UpperLimit',50);
actInfo_1.Name = 'LD Parameter';

obsInfo_2 = rlNumericSpec([2 1]);
obsInfo_2.Name = 'observations';
obsInfo_2.Description = 'Velocity and Time';
 
actInfo_2 = rlNumericSpec([1 1],'LowerLimit',1,'UpperLimit',15);
actInfo_2.Name = 'Velocity';

obsInfo = {obsInfo_1,obsInfo_2};
actInfo = {actInfo_1,actInfo_2};

agentBlks = mdl + ["/RL Agent1", "/RL Agent2"];
env = rlSimulinkEnv(mdl,agentBlks, obsInfo, actInfo);

env.ResetFcn = @(in)localResetFcn(in);

%numObservations = obsInfo_1.Dimension(1);
%numActions = prod(actInfo_1.Dimension);

rng(0)


%% Creating Agents

agent1 = createAgent_LD(obsInfo_1,actInfo_1,sampling);
agent2 = createAgent_vel(obsInfo_2,actInfo_2,sampling);

%% Agents Training

maxepisodes = 550;
maxsteps = ceil(Ts/sampling);
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'ScoreAveragingWindowLength',100, ...
    'Verbose',true, ...
    'Plots','training-progress',...
    'UseParallel',false,...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',[-1500,-1000]);

trainOpts.ParallelizationOptions.Mode = "async";

doTraining = false;

if doTraining
    % Train the agent.
    trainingStats = train([agent1,agent2],env,trainOpts);
    save("savedAgents/MultiAgentSAC1.mat",'agent1')
    save("savedAgents/MultiAgentSAC2.mat",'agent2')
else
    % Load pretrained agent for the example.
    %load('savedAgents/finalAgentSAC.mat','agent')
    load('savedAgents/MultiAgentSAC1.mat','agent1');
    load('savedAgents/MultiAgentSAC2.mat','agent2');
    disp("Trained Agents is loaded..")
end

%% Validate the trained agent

% simOpts = rlSimulationOptions('MaxSteps',maxsteps);
% experiences = sim(env,agent,simOpts);
% actor = getActor(agent);
% parameters = getLearnableParameters(actor);
% 
% Ld = abs(parameters{1}(1));
%Kp = abs(parameters{1}(2));



%% Run simulation
out = sim(mdl);
scatter(yRef,xRef,'green','filled')


function in = localResetFcn(in)

in = setVariable(in,'X_o', randi(100,1,1)-200);
in = setVariable(in,'Y_o', randi(50,1,1)-20);
in = setVariable(in,'vel', randi(15,1,1));
in = setVariable(in,'psi_o', 1.57*(-1 + 2*rand(1,1)));
in = setVariable(in,'ld', rand(1,1));

end
