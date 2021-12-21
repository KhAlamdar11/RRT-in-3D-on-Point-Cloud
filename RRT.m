close all;
figure,
ptCloud = pcread('bunny.ply');
pcshow(ptCloud);


locs = ptCloud.Location;
x = locs(:,1); 
y = locs(:,2);
z = locs(:,3);
plot3(x,y,z,'o')


map3D = occupancyMap3D(100,'OccupiedThreshold',0.2);
pose = [ 0.9 0.9 0.9 0.9 0.9 0.9 0.9];
points = locs;
%points2 = [(0:0.25:2)' (2:-0.25:0)' (0:0.25:2)'];
maxRange = 5;
insertPointCloud(map3D,pose,points,maxRange)

x = [0.5:0.075:2.25]
y = [0.25:0.075:2.5]
z = [1.5:0.075:3.5]

obs = [0.001 0.001 0.001 0.001;0.001 0.001 0.001 0.001]
indexx = 3;
show(map3D)

%%


for i=x
    for j=y
        for k=z
            if checkOccupancy(map3D,[i j k]) == 1
                obs(indexx,1) = i;obs(indexx,2) = j; obs(indexx,2) = j;
                obs(indexx,3) = k; obs(indexx,4) = 0.075;
                indexx = indexx+1;
            end
        end
    end
end

ITER = 2000; 
start_point = [0.5 1.5 2.25 0];
 
goal_point = [2.5 1.5 2.25 1000]; 
 
map = [start_point]; 
epsilon = 0.25; 
 
%obstacles: 
%obstacles = [2 3 1 2; 7 8 2 2;-1 -1 6 2;-8 -5 -2 2]; %centers and radii
obstacles = obs;
             
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
 
 plot3(start_point(1),start_point(2),start_point(3),'r*');
 hold on
 plot3(goal_point(1),goal_point(2),goal_point(3),'r*');

for i=1:ITER-1
    
    
    X_rand = 10*randn([1, 3]); %take a random point in the environment
    
    %calculate which point in the existing map has the smallest distance to
    %X_rand
    
    [smallest_idx] = closest_point(map,X_rand); 
    X_near = map(smallest_idx, 1:end-1); 
    X_new = X_near + (X_rand - X_near)/norm(X_rand - X_near)*epsilon ; %new point is proportional to the distance from the nearest point
    X_new(4) = smallest_idx;
    
    n=50;
    x_ot = zeros(n,1);
    y_ot = zeros(n,1);
    z_ot = zeros(n,1);
    object_on_line = 0; 
    for i=1:n
        x_ot(i) = X_near(1) + 1/i * (X_new(1)-X_near(1));
        y_ot(i) = X_near(2) + 1/i * (X_new(2)-X_near(2));
        z_ot(i) = X_near(3) + 1/i * (X_new(3)-X_near(3));
        
        if checkOccupancy(map3D,[x_ot(i) y_ot(i) z_ot(i)]) == 1
            object_on_line = 1; break;
        end
    end

   if object_on_line == 0
        map = [map; X_new]; 
        con = [X_near; X_new(:,1:end-1)];
        line([con(1,1), con(2,1)], [con(1,2), con(2,2)], [con(1,3), con(2,3)])
    end
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
xlim([-1 3]);ylim([-1 3]);zlim([-1 5]);

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