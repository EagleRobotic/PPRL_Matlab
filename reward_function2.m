function [out]  = reward_function2(in)
    global flag reward counter;
    ep_rew = 0;
    r1 = 0; 
    r2 = 0;
    n_wp = 15; %Number of waypoints
    threshold = 2*0.5; %Distance thereshold for waypoints
    tolerance = 10; %Tolerance for waypoints maximum distance
    flag; %Missing waypoint flag
    pose = in(1:3); %Current pose of the vehicle
    x_wp = in(4:18); %X-Axis waypoints
    y_wp = in(19:33); %Y-Axis waypoints
    vel = in(34); %Velocity of the vehicle

    ind_wp = counter; %Indices for waypoints

    x_pos = pose(1);
    y_pos = pose(2);
    theta = pose(3);
    
    error = sqrt((x_wp(ind_wp)-x_pos)^2 + (y_wp(ind_wp)-y_pos)^2);
    r2 = -error;
    if error<threshold
        r1 = 50;
        counter = counter +1;
        flag = flag + 1;
    elseif error>tolerance && (flag+1)<counter
        while(counter<=n_wp)
            counter = counter + 1;
            reward_function([pose;x_wp;y_wp;vel]); %Recursive calling
        end
    end
    ep_rew = -1
    reward(end+1) = ep_rew;
    out = mean(reward);

end