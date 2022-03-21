function [out]  = reward_function(in)
    global flag reward counter old_pos TP;
    ep_rew = 0;
    r1 = 0; %Waypoint reward
    r2 = 0; %Distance reward
    r3 = 0; %Living cost
    r4 = 0; %Reach reward
    r5 = 0; %Waypoint error
    done = 0;

    n_wp = 15; %Number of waypoints
    threshold = 8*0.5; %Distance thereshold for waypoints
    tolerance = 10; %Tolerance for waypoints maximum distance
    flag; %Missing waypoint flag
    
    pose = in(1:3); %Current pose of the vehicle
    x_wp = in(4:18); %X-Axis waypoints
    y_wp = in(19:33); %Y-Axis waypoints
    vel = in(34); %Velocity of the vehicle
    
    
    ind_wp = counter; %Indices for waypoints

    %First simulation position is zero ??????????
    x_pos = pose(1);
    y_pos = pose(2);
    theta = pose(3); 

    delta = sqrt((x_pos-old_pos(1))^2 + (y_pos-old_pos(2))^2);
    TP = TP + delta;
    old_pos = [x_pos,y_pos];
    
    error = sqrt((x_wp(ind_wp)-x_pos)^2 + (y_wp(ind_wp)-y_pos)^2); %Distance for next waypoint
    target = sqrt((x_wp(end)-x_pos)^2 + (y_wp(end)-y_pos)^2); %Distance for goal point

    r2 = -target/n_wp; %Distance reward
    r5 = -error/n_wp; %Waypoint error


    if target<=threshold
        done = 1;
        r4 = 100;
    else
        r3 = -1; %Living cost
    end

    if error<threshold
         counter = counter +1;
         flag = flag + 1;
         r1 = flag*5;
    end
         
%     elseif error>tolerance && (flag+1)<counter
%         while(counter<=n_wp)
%             counter = counter + 1;
%             reward_function([pose;x_wp;y_wp;vel]); %Recursive calling
%         end
%     end


    ep_rew = r1 + r2 + r3 + r4;
    reward(end+1) = mean(ep_rew);

    out = [sum(reward); target; mean(error); done];

end