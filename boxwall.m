clear all; close all;
figure,
for i=1:5000
    x(i) = rand*2 +3;
    y(i) = rand*2 + 3;
    z(i) = rand*5;
end

plot3(x,y,z,'o');
axis([0 5 0 5 0 5]);

figure,
map3D = occupancyMap3D(1);
pose = [ 0.5 0.5 0.5 0.5 0.5 0.5 0.5];
points = [y' z' x'];
%points = [x' y' z'];
%points2 = [(0:0.25:2)' (2:-0.25:0)' (0:0.25:2)'];
maxRange = 10;
insertPointCloud(map3D,pose,points,maxRange)

show(map3D)
%getOccupancy(map3D,(2.3,2.3,1.5));

%%
figure,
plot3(x,y,z,'o'); hold on;
axis([0 8 0 8 0 8]);

ITER = 2000; 
start_point = [0 0 0 0];
 
goal_point = [7 7 4 1000]; 

x1 = start_point(1);y1 = start_point(2);z1 = start_point(3);
x2 = goal_point(1);y2 = goal_point(2);z2 = goal_point(3);
n=10;
x = zeros(n,1);
y = zeros(n,1);
z = zeros(n,1);
for i=1:n
    x(i) = x1 + i/n * (x2-x1);
    y(i) = y1 + i/n * (y2-y1);
    z(i) = z1 + i/n * (z2-z1);
end
plot3(x,y,z,'o');hold on
plot3(x1,y1,z1,'x')
plot3(x2,y2,z2,'x')

map = [start_point]; 
epsilon = 0.25; 

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
        x_ot(i) = X_near(1) + i/n * (X_new(1)-X_near(1));
        y_ot(i) = X_near(2) + i/n * (X_new(2)-X_near(2));
        z_ot(i) = X_near(3) + i/n * (X_new(3)-X_near(3));
        
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
xlim([0 8]);ylim([0 8]);zlim([0 8]);

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