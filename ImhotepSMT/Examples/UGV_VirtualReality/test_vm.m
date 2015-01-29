clc; close all; clear all;

world = vrworld('ugv_vr.wrl');
open(world);

fig = view(world, '-internal');
vrdrawnow;

car = vrnode(world, 'Automobile');
camera = vrnode(world, 'View1');


camraCordInit = [2.4, 10.2, 25.8] - [-7.5, 0, 20];
carCordInit = [13,0.2,-11];
diffCamera = camraCordInit - carCordInit;

car.translation = carCordInit;
camera.position = car.translation + diffCamera;
car.rotation = [0, 1, 0, -pi/2];
vrdrawnow;



z1 = -10:35;
x1 = 13 * ones(size(z1));
y1 = 0.1* ones(size(z1));



x2 = 13:-1:-31;
z2 = 35 * ones(size(x2));
y2 = 0.1* ones(size(z2));

z3 = 35:-1:-10;
x3 = -31 * ones(size(z3));
y3 = 0.1* ones(size(z3));



x4 = -31:1:13;
z4 = -11 * ones(size(x4));
y4 = 0.1* ones(size(z4));

for i=1:length(x1)
    %car.translation = [x1(i) y1(i) z1(i)];
    car.translation = car.translation + [0 0 1];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, pi];
vrdrawnow;



for i=1:length(x2)
    car.translation = [x2(i) y2(i) z2(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, pi/2];
vrdrawnow;


for i=1:length(x3)
    car.translation = [x3(i) y3(i) z3(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end

car.rotation = [0, 1, 0, 0];
vrdrawnow;


for i=1:length(x4)
    car.translation = [x4(i) y4(i) z4(i)];
    camera.position = car.translation + diffCamera;
    vrdrawnow;
    pause(0.1);
end













%%
% edit(vrworld('my_vrmount.wrl'))
