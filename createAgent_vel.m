function agent = createAgent_vel(obsInfo,actInfo,sampling)


%% Creating SAC Algorithm

% Define the network layers.
cnet = [
    featureInputLayer(obsInfo.Dimension(1),"Normalization","none","Name","observation")
    fullyConnectedLayer(128,"Name","fc1")
    concatenationLayer(1,2,"Name","concat")
    reluLayer("Name","relu1")
    fullyConnectedLayer(64,"Name","fc3")
    reluLayer("Name","relu2")
    fullyConnectedLayer(32,"Name","fc4")
    reluLayer("Name","relu3")
    fullyConnectedLayer(1,"Name","CriticOutput")];
actionPath = [
    featureInputLayer(actInfo.Dimension(1),"Normalization","none","Name","action")
    fullyConnectedLayer(128,"Name","fc2")];

% Connect the layers.
criticNetwork = layerGraph(cnet);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork,"fc2","concat/in2");
%plot(criticNetwork)

criticdlnet = dlnetwork(criticNetwork,'Initialize',false);
criticdlnet1 = initialize(criticdlnet);
criticdlnet2 = initialize(criticdlnet);

critic1 = rlQValueFunction(criticdlnet1,obsInfo,actInfo, ...
    "ObservationInputNames","observation");
critic2 = rlQValueFunction(criticdlnet2,obsInfo,actInfo, ...
    "ObservationInputNames","observation");

% Create the actor network layers.
anet = [
    featureInputLayer(obsInfo.Dimension(1),"Normalization","none","Name","observation")
    fullyConnectedLayer(128,"Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(64,"Name","fc2")
    reluLayer("Name","relu2")];
meanPath = [
    fullyConnectedLayer(32,"Name","meanFC")
    reluLayer("Name","relu3")
    fullyConnectedLayer(actInfo.Dimension(1),"Name","mean")];
stdPath = [
    fullyConnectedLayer(actInfo.Dimension(1),"Name","stdFC")
    reluLayer("Name","relu4")
    softplusLayer("Name","std")];

% Connect the layers.
actorNetwork = layerGraph(anet);
actorNetwork = addLayers(actorNetwork,meanPath);
actorNetwork = addLayers(actorNetwork,stdPath);
actorNetwork = connectLayers(actorNetwork,"relu2","meanFC/in");
actorNetwork = connectLayers(actorNetwork,"relu2","stdFC/in");
%plot(criticNetwork)

actordlnet = dlnetwork(actorNetwork);
actor = rlContinuousGaussianActor(actordlnet, obsInfo, actInfo, ...
    "ObservationInputNames","observation", ...
    "ActionMeanOutputNames","mean", ...
    "ActionStandardDeviationOutputNames","std");

agentOpts = rlSACAgentOptions( ...
    "SampleTime",sampling, ...
    "TargetSmoothFactor",1e-3, ...    
    "ExperienceBufferLength",1e8, ...
    "MiniBatchSize",128, ...
    "NumWarmStartSteps",1000, ...
    "DiscountFactor",0.99);

agentOpts.ActorOptimizerOptions.Algorithm = "adam";
agentOpts.ActorOptimizerOptions.LearnRate = 1e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;

for ct = 1:2
    agentOpts.CriticOptimizerOptions(ct).Algorithm = "adam";
    agentOpts.CriticOptimizerOptions(ct).LearnRate = 1e-4;
    agentOpts.CriticOptimizerOptions(ct).GradientThreshold = 1;
end

agent = rlSACAgent(actor,[critic1,critic2],agentOpts);