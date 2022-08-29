clear
clc
close all

T = 0:0.1:10;
n = length(T);
theta1 = zeros(n,1);
theta2 = zeros(n,1);
theta3 = zeros(n,1);

%% Rotation around theta1

for i = 1: n
    theta1(i) = pi/3+pi/300*i;
    theta2(i) = pi/4;
    theta3(i) = pi/2;
end
mode1 = [0.5, 8, 10];
plotenergy(T,theta1,theta2,theta3,mode1);

%% Rotation around theta2

for i = 1: n
    theta1(i) = 3*pi/5;
    theta2(i) = pi/12+pi/300*i;
    theta3(i) = pi/2;
end
mode2 = [0.5, 5, 10];
plotenergy(T,theta1,theta2,theta3,mode2);

%% Rotation around theta3

for i = 1: n
    theta1(i) = 3*pi/5;
    theta2(i) = pi/3;
    theta3(i) = pi/5 + pi/202*i;
end
mode3 = [-5, 1, 5];
plotenergy(T,theta1,theta2,theta3,mode3);

%% mixed Rotation

for i = 1: n
    theta1(i) = 2*pi/5+pi/500*i;
    theta2(i) = pi/6+pi/400*i;
    theta3(i) = pi/4 + pi/300*i;
end
mode4 = [-5, 1, 5];
plotenergy(T,theta1,theta2,theta3,mode4);


function plotenergy(T,theta1,theta2,theta3,mode)
n = length(T);
V = zeros(n,1);
VE = zeros(n,1);
VG = zeros(n,1);
for i = 1:n
    [VE(i),VG(i)] = plotRobot(theta1(i),theta2(i),theta3(i),mode);
    V(i) = VE(i)+VG(i);
    drawnow
    if i > 1
        pause(T(i)-T(i-1))
    end
    clf
end

plot(T,VE,'--','linewidth',1.5);
hold on
plot(T,VG,'--','linewidth',1.5);
hold on
plot(T,V,'linewidth',1.5);
xlabel('Time');
ylabel('Potential Energy');
legend('V_E','V_G','V_{Total}','location','best');
title('Elastic, Gravitational and Total Potential Energy')
ylim([0 2.5*10^4]);

end

function [V_E,V_G] = plotRobot(theta1,theta2,theta3,mode)
m = 8; m1 = 1; m2 = 1; m3 = 1; m4 = 1;
l1 = 160; l2 = 80; l3 = 160; l4 = 240; b1 = 40;
d1 = 120; d2 = 100; g = 9.8;
k1 = (m1*g*(l1+b1)/2+m2*g*l1+m3*g*l3/2+m/4*g*l1)/(b1*d1);
k2 = (m1*g*(l4-l2)+m2*g*(l4-l2/2)+m3*g*l4+m4*g*l4/2+m/4*g*(l4-l2))/(l2*d2);

%point A pos
A = [0; 0; l1*sin(theta3)*sin(theta1)+(l4-l2)*sin(theta3)*sin(theta2)];
%point B pos
B = [-l2*sin(theta3)*cos(theta2); -l2*cos(theta3); l1*sin(theta3)*sin(theta1)+l4*sin(theta3)*sin(theta2)];
%point C pos
C = B + [l3*sin(theta3)*cos(theta1); -l3*cos(theta3); -l3*sin(theta3)*sin(theta1)];
%point D pos
D = [l1*sin(theta3)*cos(theta1); -l1*cos(theta3) ; (l4-l2)*sin(theta3)*sin(theta2)];
%point E pos
E = D + [(l4-l2)*sin(theta3)*cos(theta2); (l4-l2)*cos(theta3); -(l4-l2)*sin(theta3)*sin(theta2)];
%point F pos
F = A + [-b1*sin(theta3)*cos(theta1); b1*cos(theta3); b1*sin(theta3)*sin(theta1)];
%point G pos
G = A + [0; 0; d2];
%point H pos
H = A + [0; 0; d1];


r1 = [A B].';
r2 = [A D].';
r3 = [B C].';
r4 = [C D].';
r5 = [D E].';
r6 = [A F].';
r7 = [F H].';
r8 = [B G].';

plot3(r1(:,1),r1(:,2),r1(:,3),'linewidth',2,'color','b');
hold on;
plot3(r2(:,1),r2(:,2),r2(:,3),'linewidth',2,'color','b');
hold on;
plot3(r3(:,1),r3(:,2),r3(:,3),'linewidth',2,'color','b');
hold on;
plot3(r4(:,1),r4(:,2),r4(:,3),'linewidth',2,'color','b');
hold on;
plot3(r5(:,1),r5(:,2),r5(:,3),'linewidth',2,'color','b');
hold on;
plot3(r6(:,1),r6(:,2),r6(:,3),'linewidth',2,'color','b');
hold on;
plot3(r7(:,1),r7(:,2),r7(:,3),'linewidth',2,'color','r');
hold on;
plot3(r8(:,1),r8(:,2),r8(:,3),'linewidth',2,'color','r');
hold on;

plot3(A(1),A(2),A(3),'k.','markersize',25);
hold on;
plot3(B(1),B(2),B(3),'k.','markersize',25);
hold on;
plot3(C(1),C(2),C(3),'k.','markersize',25);
hold on;
plot3(D(1),D(2),D(3),'k.','markersize',25);
hold on;
plot3(E(1),E(2),E(3),'k.','markersize',25);
hold on;
plot3(F(1),F(2),F(3),'k.','markersize',25);
hold on;
plot3(G(1),G(2),G(3),'k.','markersize',25);
hold on;
plot3(H(1),H(2),H(3),'k.','markersize',25);

axis([-300 300 -300 300 -50 450]);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(mode)
str1 = strcat('theta1 = ',num2str(theta1));
str2 = strcat('theta2 = ',num2str(theta2));
str3 = strcat('theta3 = ',num2str(theta3));
text(250,-250,400,{str1});
text(250,-250,340,{str2});
text(250,-250,280,{str3});

%Elastic energy
V_E = 0.5*k1*(b1^2+d1^2)+0.5*k2*(l2^2+d2^2)-k1*b1*d1*sin(theta1)*sin(theta3)-k2*d2*l2*sin(theta2)*sin(theta3);
%gravity energy
V_G = (m1*g*(l1+b1)/2+m2*g*l1+m3*g*l3/2+m/4*g*l1)*sin(theta1)*sin(theta3)...
     +(m1*g*(l4-l2)+m2*g*(l4-l2/2)+m3*g*l4+m4*g*l4/2+m/4*g*(l4-l2))*sin(theta2)*sin(theta3);
end
