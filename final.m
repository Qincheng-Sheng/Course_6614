clear
clc
close all

global times;
times = 1;
lower_L = 1.0;
upper_L = 1.0;
tri_len = 1.5;
b1 = [tri_len/2,0,0];
b2 = [-tri_len/2,0,0];
b3 = [0, tri_len*sqrt(3)/2, 0];
base = [b1;b2;b3];

for i = 1:4
    phi = 0;
    phi2 = linspace(0,0,10);
    H = linspace(0.5,1.8,10);
    [q1,q2,q3,ee] = eemove(base,lower_L,upper_L,tri_len,phi2,H,phi);
    base = basemove(ee,q1,q2,q3,lower_L,upper_L,tri_len,phi2,H,phi);
end

for i = 1:1
    phi2 = linspace(0,45*pi/180,10);
    H = linspace(0.5,0.5,10);
    [q1,q2,q3,ee] = eemove(base,lower_L,upper_L,tri_len,phi2,H,phi);
    base = basemove(ee,q1,q2,q3,lower_L,upper_L,tri_len,phi2,H,phi);
    phi = phi+phi2(10);
end


for i = 1:3
    phi2 = linspace(0,0,10);
    H = linspace(0.5,1.0,10);
    [q1,q2,q3,ee] = eemove(base,lower_L,upper_L,tri_len,phi2,H,phi);
    base = basemove(ee,q1,q2,q3,lower_L,upper_L,tri_len,phi2,H,phi);
end

function [q1,q2,q3,ee] = eemove(base,lower_L,upper_L,tri_len,phi2,H,phi)
    q1 = zeros(10,1); q2 = zeros(10,1); q3 = zeros(10,1);
    for i = 1:10
            [j1,j2,j3] = caljoint(tri_len,upper_L,phi2(i),H(i));
            q1(i) = j1; q2(i) = j2; q3(i) = j3;
            [b,e,m] = calpos(base, lower_L, tri_len,j1,j2,j3,phi2(i),H(i),phi);
        plotRobot(b,m,e,upper_L,tri_len);
        drawnow
        if i > 1
            pause(0.1);
        end
        clf
    end
    ee = e;
end

function base = basemove(ee,q1,q2,q3,lower_L,upper_L,tri_len,phi2,H,phi)
    for i = linspace(10,1,10)
        j1 = - q1(i); j2 = -q2(i); j3 = - q3(i); 
        [b,e,m] = calpos(ee,lower_L,tri_len,j1,j2,j3,-phi2(i),-H(i),phi);
        plotRobot(b,m,e,upper_L,tri_len);
        drawnow
        if i > 1
            pause(0.1);
        end
        clf
    end
    base = e;
end

function [j1,j2,j3] = caljoint(tri_len,upper_L,phi2,H)
    % this function calaulate the joint angle
    j1 = asin(H/2*upper_L);
    j2 = asin(H/2*upper_L);
    
    a = 2*(H+tri_len*sqrt(3)/2*sin(phi2));
    b = -2*(tri_len*sqrt(3)/2-tri_len*sqrt(3)/2*cos(phi2));
    c = (-b/2)^2 + (a/2)^2;
    
    if a^2-c^2+b^2 < 0
        disp('math error');
    end
    j31 = 2*atan((a+sqrt(a^2-c^2+b^2))/(c+b));
    j32 = 2*atan((a-sqrt(a^2-c^2+b^2))/(c+b));
    if j31 < pi/2 && j31> - pi/2
        j3 = j31;
    else
        j3 = j32;
    end
end

function [b,e,m] = calpos(base,l,tri_len,j1,j2,j3,phi2,H,phi)
    j2 = pi - j2;
    R = [1 0 0;0 cos(-phi) -sin(-phi);0 sin(-phi) cos(-phi)];
    b1 = R*base(1,:)'; b2 = R*base(2,:)'; b3 = R*base(3,:)';
    b = base;
    
    
    % m1, m2 ,m3
    posm1 = [b1(1)+l*cos(j1), b1(2),  b1(3)+l*sin(j1)];
    posm2 = [b2(1)+l*cos(j2), b2(2),  b2(3)+l*sin(j1)];
    posm3 = [b3(1), b3(2)+l*cos(j3), b3(3)+l*sin(j3)];
    R = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    posm1 = R*posm1'; posm2 = R*posm2'; posm3 = R*posm3';
    m = [posm1';posm2';posm3'];
    
    % e1, e2, e3
    e1 = [b1(1),b1(2),b1(3)+H];
    e2 = [b2(1),b2(2),b2(3)+H];
    e3 = [b3(1),b3(2)+tri_len*sqrt(3)/2*cos(phi2)-tri_len * sqrt(3)/2, ...
          b3(3)+tri_len*sqrt(3)/2*sin(phi2)+H];   
    e1 = R*e1'; e2 = R*e2'; e3 = R*e3';
    e = [e1';e2';e3'];
end

function plotRobot(b,m,e,upper_l,tri_len)
    global times;
    plot_curvetube(tri_len);
    
    b1 = b(1,:); b2 = b(2,:); b3 = b(3,:);
    plot3([b1(1) b2(1)],[b1(2) b2(2)],[b1(3) b2(3)],'b');
    hold on;
    plot3([b1(1) b3(1)],[b1(2) b3(2)],[b1(3) b3(3)],'b');
    plot3([b2(1) b3(1)],[b2(2) b3(2)],[b2(3) b3(3)],'b');
    plot3(b1(1),b1(2),b1(3),'k.','markersize',25);
    plot3(b2(1),b2(2),b2(3),'k.','markersize',25);
    plot3(b3(1),b3(2),b3(3),'k.','markersize',25);
    
    % end -effector 
    e1 = e(1,:); e2 = e(2,:); e3 = e(3,:);
    plot3([e1(1) e2(1)],[e1(2) e2(2)],[e1(3) e2(3)],'b');
    plot3([e1(1) e3(1)],[e1(2) e3(2)],[e1(3) e3(3)],'b');
    plot3([e2(1) e3(1)],[e2(2) e3(2)],[e2(3) e3(3)],'b');
    plot3(e1(1),e1(2),e1(3),'k.','markersize',25);
    plot3(e2(1),e2(2),e2(3),'k.','markersize',25);
    plot3(e3(1),e3(2),e3(3),'k.','markersize',25);
    
    %lower-link
    posm1 = m(1,:); posm2 = m(2,:); posm3 = m(3,:);
    plot3([b1(1) posm1(1)],[b1(2) posm1(2)],[b1(3) posm1(3)],'r');
    plot3([b2(1) posm2(1)],[b2(2) posm2(2)],[b2(3) posm2(3)],'r');
    plot3([b3(1) posm3(1)],[b3(2) posm3(2)],[b3(3) posm3(3)],'r');
    plot3(posm1(1),posm1(2),posm1(3),'k.','markersize',25);
    plot3(posm2(1),posm2(2),posm2(3),'k.','markersize',25);
    plot3(posm3(1),posm3(2),posm3(3),'k.','markersize',25);
    
    %upper-link % check if the upper-link consistent
    plot3([e1(1) posm1(1)],[e1(2) posm1(2)],[e1(3) posm1(3)],'g');
    plot3([e2(1) posm2(1)],[e2(2) posm2(2)],[e2(3) posm2(3)],'g');
    plot3([e3(1) posm3(1)],[e3(2) posm3(2)],[e3(3) posm3(3)],'g');
    if abs(norm(e1 - posm1)-upper_l) < 1e-3 && abs(norm(e2 - posm2)-upper_l) < 1e-3 && abs(norm(e3 - posm3)-upper_l) < 1e-3
    else
        disp('error');
    end 
    axis([-3 15 -3 15 -3 15]);
%     axis equal;
%     view(90,0)
    MakeGif('climb curve pipe.gif',times);
    times = times + 1;
    hold off
end

function plot_straighttube(tri_len)
    t_step = 0.1; 
    x = zeros(100,1);
    y = zeros(100,1);
    z = zeros(100,1);
    z_height = 0.5; %m
    i = 1;
    for t = 0:t_step:12
        x(i) = 0;
        y(i) = 0;
        z(i) = z_height * t/t_step;
        i = i+1;
    end 
    y = y + tri_len*sqrt(3)/4;
    plot3(x,y,z,'LineWidth',5);
    hold on
end

function plot_curvetube(tri_len)
    t_step = 0.1; 
    x = zeros(40,1);
    y = zeros(40,1);
    z = zeros(40,1);
    z_height = 0.5; %m
    curve_angle = pi/4; 
    radius = 2;
    pipe_angle =  pi/4;
    straight_length = 5; %10m

    % for this 45deg curve, we climb 9 times, each time with 5 deg diff 
    i = 2;
    j = 1; 
    while i <= 4/t_step %run for 4s  
        if z_height * (i-1) <= straight_length
            x(i) = 0;
            y(i) = 0;
            z(i) = z_height + z(i-1);
        elseif z_height * (i-1) > straight_length
            z(i) = straight_length + radius * sin(5/180 * pi * j);
            y(i) = radius * cos(5/180 * pi * j) - radius;
            x(i) = 0; 
            j = j+1;

            %z_a3 = sin(phi2) * length(三角形中线长度) + z(i-1)
            if  z(i-1) >= straight_length + cos(curve_angle) * radius 
                x(i) = 0;
                y(i) = y(i-1) - z_height * cos(pipe_angle);
                z(i) = z(i-1) + z_height * sin(pipe_angle);
            end 
        end   
        i = i + 1; 
    end 

    y = y + tri_len*sqrt(3)/6;
    plot3(x,y,z,'LineWidth',5);
    hold on
end

function MakeGif(filename,index)  
    f = getframe(gcf);  
    imind = frame2im(f);  
    [imind,cm] = rgb2ind(imind,256);  
    if index==1  
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.001);
    else  
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.001);
    end
end
