%Link Lengths
l1 = 5; l2 = 3;

%Trajectory Parameters
a = 4; b = 2;

axis(gca, 'equal');
axis([-10 10 -10 10]);
grid on;

w = 0.1;
%loop
for t=1:1000
    %End effector speed & Coordinates
    xe = cos(w*t)*a;
    ye = sin(w*t)*b -6.9;
    syms theta1;
    syms theta2;
    
    %Trajectory center coordinates
    P4 = [0,-6.9];
    
    %HipCoordinate
    P1 = [0,0];
    
    
    %End Effector Trajectory
    if(ye< -6.9)
        ye= -6.9;
    end
    P3 = [xe ye];
    
    %line1 = line([P1(1) P3(1)],[P1(2) P3(2)]);
    Traj_identi = viscircles(P3,0.1);
    
    
    
    %End Effector coord.
    x = l1*cos(theta1) + l2*cos(theta1 + theta2);
    y = l1*sin(theta1) + l2*sin(theta1 + theta2);
    
    %Joint velocities values
    J = jacobian([x,y], [theta1, theta2]);
    
    
   if(atan(ye/xe)>0)
        alpha = atan(ye/xe)-pi;
    else
        alpha = atan(ye/xe);
    end
    
    theta1 = cosine_rule(l2,l1,sqrt(xe^2+ye^2))+alpha;
    theta2 = -pi+cosine_rule(sqrt(xe^2+ye^2),l1,l2);
    
    l1*cos(theta1)+l2*cos(theta1+theta2)
    l1*sin(theta1)+l2*sin(theta1+theta2)
    
    radtodeg(cosine_rule(sqrt(xe^2+ye^2),l1,l2));
    %theta1
    %theta2
    
    %knee coordinates
    P2 = [l1*cos(theta1) , l1*sin(theta1)];
    x = l1*cos(theta1) + l2*cos(theta1 + theta2);
    y = l1*sin(theta1) + l2*sin(theta1 + theta2);
    P4 = [x y];
    theta2;
    theta1;
    line2 = line([P1(1) P2(1)],[P1(2) P2(2)]);
    line3 = line([P3(1) P2(1)],[P3(2) P2(2)]);
    line4 = line([P4(1) P2(1)],[P4(2) P2(2)]);
    
    %vel = inv(J)*[-sin(t)*a; cos(t)*b];
    %Update Interval;
    pause(0.001);
    %remove previous line
    delete(line2);
    delete(line3);
    delete(line4);
    %delete(Traj_identifier); 
end