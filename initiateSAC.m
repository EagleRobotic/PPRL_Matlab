close all; clear; clc;

%mdl = 'new_model';
mdl = 'exercise_modelv2';
open_system(mdl)
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
Y_o = -refPose(1,2)+0; % initial vehicle y position
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

vel = 15; %m/s 
L = 3; % bicycle length
ld = 5; % lookahead distance

% %% RL Spesifications
% 
% obsInfo = rlNumericSpec([3 1]);
% obsInfo.Name = 'observations';
% obsInfo.Description = 'Integrated CTE, CTE, velocity';
% 
% actInfo = rlNumericSpec([2 1], ...
%                         'LowerLimit',0.5,'UpperLimit',50, ...
%                         'LowerLimit',1,'UpperLimit',15);
% 
% actInfo.Name = 'LD Parameter and velocity';
% 
% agentBlk = [mdl '/RL Agent'];
% env = rlSimulinkEnv(mdl,agentBlk, obsInfo, actInfo);
% 
% env.ResetFcn = @(in)localResetFcn(in);
% 
% numObservations = obsInfo.Dimension(1);
% numActions = prod(actInfo.Dimension);
% 
% rng(0)
% 
% %% Creating SAC Algorithm
% 
% % Define the network layers.
% cnet = [
%     featureInputLayer(numObservations,"Normalization","none","Name","observation")
%     fullyConnectedLayer(128,"Name","fc1")
%     concatenationLayer(1,2,"Name","concat")
%     reluLayer("Name","relu1")
%     fullyConnectedLayer(64,"Name","fc3")
%     reluLayer("Name","relu2")
%     fullyConnectedLayer(32,"Name","fc4")
%     reluLayer("Name","relu3")
%     fullyConnectedLayer(1,"Name","CriticOutput")];
% actionPath = [
%     featureInputLayer(numActions,"Normalization","none","Name","action")
%     fullyConnectedLayer(128,"Name","fc2")];
% 
% % Connect the layers.
% criticNetwork = layerGraph(cnet);
% criticNetwork = addLayers(criticNetwork, actionPath);
% criticNetwork = connectLayers(criticNetwork,"fc2","concat/in2");
% %plot(criticNetwork)
% 
% criticdlnet = dlnetwork(criticNetwork,'Initialize',false);
% criticdlnet1 = initialize(criticdlnet);
% criticdlnet2 = initialize(criticdlnet);
% 
% critic1 = rlQValueFunction(criticdlnet1,obsInfo,actInfo, ...
%     "ObservationInputNames","observation");
% critic2 = rlQValueFunction(criticdlnet2,obsInfo,actInfo, ...
%     "ObservationInputNames","observation");
% 
% % Create the actor network layers.
% anet = [
%     featureInputLayer(numObservations,"Normalization","none","Name","observation")
%     fullyConnectedLayer(128,"Name","fc1")
%     reluLayer("Name","relu1")
%     fullyConnectedLayer(64,"Name","fc2")
%     reluLayer("Name","relu2")];
% meanPath = [
%     fullyConnectedLayer(32,"Name","meanFC")
%     reluLayer("Name","relu3")
%     fullyConnectedLayer(numActions,"Name","mean")];
% stdPath = [
%     fullyConnectedLayer(numActions,"Name","stdFC")
%     reluLayer("Name","relu4")
%     softplusLayer("Name","std")];
% 
% % Connect the layers.
% actorNetwork = layerGraph(anet);
% actorNetwork = addLayers(actorNetwork,meanPath);
% actorNetwork = addLayers(actorNetwork,stdPath);
% actorNetwork = connectLayers(actorNetwork,"relu2","meanFC/in");
% actorNetwork = connectLayers(actorNetwork,"relu2","stdFC/in");
% %plot(criticNetwork)
% 
% actordlnet = dlnetwork(actorNetwork);
% actor = rlContinuousGaussianActor(actordlnet, obsInfo, actInfo, ...
%     "ObservationInputNames","observation", ...
%     "ActionMeanOutputNames","mean", ...
%     "ActionStandardDeviationOutputNames","std");
% 
% agentOpts = rlSACAgentOptions( ...
%     "SampleTime",sampling, ...
%     "TargetSmoothFactor",1e-3, ...    
%     "ExperienceBufferLength",1e8, ...
%     "MiniBatchSize",128, ...
%     "NumWarmStartSteps",1000, ...
%     "DiscountFactor",0.99);
% 
% agentOpts.ActorOptimizerOptions.Algorithm = "adam";
% agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
% agentOpts.ActorOptimizerOptions.GradientThreshold = 1;
% 
% for ct = 1:2
%     agentOpts.CriticOptimizerOptions(ct).Algorithm = "adam";
%     agentOpts.CriticOptimizerOptions(ct).LearnRate = 1e-4;
%     agentOpts.CriticOptimizerOptions(ct).GradientThreshold = 1;
% end
% 
% agent = rlSACAgent(actor,[critic1,critic2],agentOpts);
% 
% %% Agent Training
% 
% maxepisodes = 550;
% maxsteps = ceil(Ts/sampling);
% trainOpts = rlTrainingOptions(...
%     'MaxEpisodes',maxepisodes, ...
%     'MaxStepsPerEpisode',maxsteps, ...
%     'ScoreAveragingWindowLength',100, ...
%     'Verbose',true, ...
%     'Plots','training-progress',...
%     'UseParallel',false,...
%     'SaveAgentCriteria',"AverageReward",'SaveAgentValue',-1500,...
%     'StopTrainingCriteria','AverageReward',...
%     'StopTrainingValue',-1500);
% 
% trainOpts.ParallelizationOptions.Mode = "async";
% 
% doTraining = false;
% 
% if doTraining
%     % Train the agent.
%     trainingStats = train(agent,env,trainOpts);
%     save("savedAgents/finalAgentSACv2.mat",'agent')
% else
%     % Load pretrained agent for the example.
%     %load('savedAgents/finalAgentSAC.mat','agent')
%     load("savedAgents/finalAgentSACv2.mat",'agent')
%     disp("Have a nice day")
%     simOpts = rlSimulationOptions('MaxSteps',maxsteps);
%     experiences = sim(env,agent,simOpts);
% end

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


% function in = localResetFcn(in)
% 
% in = setVariable(in,'X_o', randi(100,1,1)-200);
% in = setVariable(in,'Y_o', randi(50,1,1)-20);
% in = setVariable(in,'vel', randi(15,1,1));
% in = setVariable(in,'psi_o', 1.57*(-1 + 2*rand(1,1)));
% in = setVariable(in,'ld', rand(1,1));
% 
% end
