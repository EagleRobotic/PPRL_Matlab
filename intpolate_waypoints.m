
load("data/test_track.mat");
waypoints = data.RoadSpecifications.Centers;
dubinsSpace = stateSpaceDubins([min(waypoints(:,1)) max(waypoints(:,1)); min(waypoints(:,2)) max(waypoints(:,2)); 0 0]);
pathobj = navPath(dubinsSpace);
append(pathobj, waypoints);
interpolate(pathobj, 250);

figure;
grid on;
axis equal;
hold on;
plot(pathobj.States(:,1), pathobj.States(:,2), ".b");
plot(waypoints(:,1), waypoints(:,2), "*r", "MarkerSize", 10)