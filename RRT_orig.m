clc; clear 
ITER = 2000; 
 
start_point = [-7 5 3 0];
 
goal_point = [7 7 4 1000]; 
 
map = [start_point]; 
epsilon = 0.5; 
 
%obstacles: 
%{
obstacles = [2 3 1 2; 
             7 8 2 2;
             -1 -1 6 2;
               -8 -5 -2 2]; %centers and radi
for i=0:10
    for j=0:10
        obstacles(idxx,1) = 0;
        obstacles(idxx,2) = i
        obstacles(idxx,3) = j;
        obstacles(idxx,4) = 1;
        idxx=idxx+1;
    end
end

%}
obstacles = [0 0 0 0.1];
idxx = 1

figure
noOfObs = size(obstacles);
for i = 1:noOfObs(1)
    [x,y,z] = sphere;
    % Scale to desire radius.
    radius = obstacles(i,4);
    x = x * radius;
    y = y * radius;
    z = z * radius;
    % Plot as surface.
    surf(x+obstacles(i,1),y+obstacles(i,2),z+obstacles(i,3))
    hold on
end
 
 plot3(start_point(1),start_point(2),start_point(3),'r*')
 hold on
 plot3(goal_point(1),goal_point(2),goal_point(3),'r+')

for i=1:ITER-1
    
    
    X_rand = 10*randn([1, 3]); %take a random point in the environment
    
    %calculate which point in the existing map has the smallest distance to
    %X_rand
    
    [smallest_idx] = closest_point(map,X_rand); 
    X_near = map(smallest_idx, 1:end-1); 
    X_new = X_near + (X_rand - X_near)/norm(X_rand - X_near)*epsilon ; %new point is proportional to the distance from the nearest point
    X_new(4) = smallest_idx;
   
    %dis_from_obs = sqrt((X_new(1)-obstacles(:, 1)).^2 + (X_new(2)-obstacles(:, 2)).^2 + (X_new(3)-obstacles(:, 3)).^2); 
    %if sum(dis_from_obs > obstacles(:, end))==4
        map = [map; X_new]; 
        con = [X_near; X_new(:,1:end-1)];
        line([con(1,1), con(2,1)], [con(1,2), con(2,2)], [con(1,3), con(2,3)])
      
    %end
end
 
waypoint_index = [];
s = size(map);
[near_goal] = closest_point(map, goal_point(:,1:3))
parent  = near_goal;
 
while parent~=0
    waypoint_index = [parent waypoint_index];
    parent = map(parent, 4);
end
waypoints = [];
for i = 1: length(waypoint_index)
    waypoint_index(i)
    waypoints(i,1) = map(waypoint_index(i),1)
    waypoints(i,2) = map(waypoint_index(i),2)
    waypoints(i,3) = map(waypoint_index(i),3)
end
waypoints(i+1,1) = goal_point(1);
waypoints(i+1,2) = goal_point(2);
waypoints(i+1,3) = goal_point(3);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3),'Color', 'r', 'LineWidth', 4 )
    
function [smallest_idx] = closest_point(map, X_i)
    x = size(map); 
    d_min = 100; 
 
    smallest_idx = 1; 
 
    for j = 1:x(1) 
 
        d = norm(map(j,1:end-1)-X_i); 
 
        %check for obstacle collision: 
 
        if d<d_min 
            smallest_idx = j;
            d_min = d; 
        end
    end
end
