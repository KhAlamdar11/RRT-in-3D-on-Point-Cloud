clc; clear; close all;
ptCloud = pcread('bunny.ply');
%pcshow(ptCloud);
figure,
locs = ptCloud.Location;
x = locs(:,1); 
y = locs(:,2);
z = locs(:,3);

map3D = occupancyMap3D(10);
pose = [ 1.5 1.5 1.5 1.5 1.5 1.5 1.5];
points = locs;
%points2 = [(0:0.25:2)' (2:-0.25:0)' (0:0.25:2)'];
maxRange = 5;
insertPointCloud(map3D,pose,points,maxRange)
show(map3D); hold on
%getOccupancy(map3D)
plot3D(5,5,0)

%%
x = [0.5:0.075:2.25]
y = [0.25:0.075:2.5]
z = [1.5:0.075:3.5]

obs = [0.001 0.001 0.001 0.001;0.001 0.001 0.001 0.001]
indexx = 3;
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