x1 = 1.0;y1 = 1.0;z1 = 5.0;
x2 = 3.0;y2 = 5.0;z2 = 7.0;
close all;
n=10;
x = zeros(n,1);
y = zeros(n,1);
z = zeros(n,1);
for i=1:n
    x(i) = x1 + i/n * (x2-x1);
    y(i) = y1 + i/n * (y2-y1);
    z(i) = z1 + i/n * (z2-z1);
end
figure,
plot3(x,y,z,'o');hold on
plot3(x1,y1,z1,'x')
plot3(x2,y2,z2,'x')

%%

indexx = 3;

for i=0:0.1:10
    for j=0:0.1:10
        for k=0:0.1:10
            if checkOccupancy(map3D,[x(i) y(j) z(k)]) == 1
                x_new(indexx) = x(i); 
                y_new(indexx) = y(j);
                z_new(indexx) = z(k);
            end
        end
    end
end
