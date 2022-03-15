function [out]  = observations(in)
    pose = in(31:33); %Current pose of the vehicle
    x_wp = in(1:15); %X-Axis waypoints
    y_wp = in(16:30); %Y-Axis waypoints
    delta = in(34); %Velocity of the vehicle
    out = delta;

end