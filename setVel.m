function setVel(in)
%SETLOOKAD Summary of this function goes here
%   Detailed explanation goes here

set_param('exercise_modelv2/Pure Pursuit Controller/Pure Pursuit','DesiredLinearVelocity',num2str(in));
set_param('exercise_modelv2/velocity','Value',num2str(in));

%set_param('MultiAgentModel/Pure Pursuit Controller/Pure Pursuit','DesiredLinearVelocity',num2str(in));
%set_param('MultiAgentModel/velocity','Value',num2str(in));
end

