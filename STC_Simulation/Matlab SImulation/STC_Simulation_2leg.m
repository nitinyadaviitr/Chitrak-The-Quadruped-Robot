%Link Lengths
l1 = 5; l2 = 3;

%Trajectory Parameters
a = 3; b = 2;

axis(gca, 'equal');
axis([-60 15 -7 20]);
grid on;

w = 0.1;
%loop
for t=0:1000
    syms theta1;
    syms theta2;
    syms theta3;
    syms theta4;
    
    %Leg1
    %End effector speed & Coordinates
    xe = cos(w*t)*a;
    ye = sin(w*t)*b -6.9;
    %Trajectory center coordinates
    P4 = [0,-6.9];
    %HipCoordinate
    P1 = [0,0];
    %End Effector Trajectory
    if(ye< -6.9)
        ye= -6.9;
    end
   if(atan(ye/xe)>0)
        alpha = atan(ye/xe)-pi;
    else
        alpha = atan(ye/xe);
    end
    
    theta1 = cosine_rule(l2,l1,sqrt(xe^2+ye^2))+alpha;
    theta2 = -pi+cosine_rule(sqrt(xe^2+ye^2),l1,l2);
    
    %knee coordinates
    P2 = [l1*cos(theta1) , l1*sin(theta1)];
    x = l1*cos(theta1) + l2*cos(theta1 + theta2);
    y = l1*sin(theta1) + l2*sin(theta1 + theta2);
    P3 = [x y];
    
    %Leg2
    %End effector speed & Coordinates
    xe1 = cos(w*t/3 -pi/3)*a + 11;
    ye1 = sin(w*t/3 -pi/3)*b -6.9;
    %Trajectory center coordinates
    P8 = [11,-6.9];
    %HipCoordinate
    P5 = [11,0];
    %End Effector Trajectory
    if(ye1< -6.9)
        ye1= -6.9;
    end
    
   if(atan(ye1/(xe1 -11))>0)
        alpha = atan(ye1/(xe1 -11))-pi;
    else
        alpha = atan(ye1/(xe1 -11));
   end
    
    theta3 = cosine_rule(l2,l1,sqrt( (xe1 - 11)^2 + ye1^2))+alpha;
    theta4 = -pi+cosine_rule(sqrt((xe1-11)^2+ye1^2),l1,l2);
    
    %knee coordinates
    P6 = [l1*cos(theta3) + 11 , l1*sin(theta3)];
    x1 = 11+l1*cos(theta3) + l2*cos(theta3 + theta4);
    y1 = l1*sin(theta3) + l2*sin(theta3 + theta4);
    P7 = [x1 y1];
    
    %Leg3
    %End effector speed & Coordinates
    xe2 = cos(w*t/3 -2*pi/3)*a;
    ye2 = sin(w*t/3 -2*pi/3)*b -6.9;
    %Trajectory center coordinates
    P12 = [0,-6.9];
    %HipCoordinate
    P9 = [0,0];
    %End Effector Trajectory
    if(ye2< -6.9)
        ye2= -6.9;
    end
    
   if(atan(ye2/(xe2))>0)
        alpha = atan(ye2/(xe2))-pi;
    else
        alpha = atan(ye2/(xe2));
   end
    
    theta5 = cosine_rule(l2,l1,sqrt( (xe2)^2 + ye2^2))+alpha;
    theta6 = -pi+cosine_rule(sqrt(xe2^2+ye2^2),l1,l2);
    
    %knee coordinates
    P10 = [l1*cos(theta5) , l1*sin(theta5)];
    x2 = l1*cos(theta5) + l2*cos(theta5 + theta6);
    y2 = l1*sin(theta5) + l2*sin(theta5 + theta6);
    P11 = [x2 y2];
    
    %Leg4
    %End effector speed & Coordinates
    xe3 = cos(w*t/3 -pi)*a+11;
    ye3 = sin(w*t/3 -pi)*b -6.9;
    %Trajectory center coordinates
    P16 = [11,-6.9];
    %HipCoordinate
    P13 = [11,0];
    %End Effector Trajectory
    if(ye3< -6.9)
        ye3= -6.9;
    end
    
   if(atan(ye3/(xe3 -11))>0)
        alpha = atan(ye3/(xe3-11))-pi;
    else
        alpha = atan(ye3/(xe3-11));
   end
    
    theta7 = cosine_rule(l2,l1,sqrt( (xe3-11)^2 + ye3^2))+alpha;
    theta8 = -pi+cosine_rule(sqrt((xe3-11)^2+ye3^2),l1,l2);
    
    %knee coordinates
    P14 = [l1*cos(theta7) + 11 , l1*sin(theta7)];
    x3 = 11+l1*cos(theta7) + l2*cos(theta7 + theta8);
    y3 = l1*sin(theta7) + l2*sin(theta7 + theta8);
    P15 = [x3 y3];
    
    
    
%     Traj_identi = viscircles(P3,0.1);
%     Traj_identi1 = viscircles(P7,0.1);
    %knee coordinates

    
    line2 = line([P1(1) P2(1)],[P1(2) P2(2)] , 'LineWidth',3 ,'Color','red');
    line3 = line([P3(1) P2(1)],[P3(2) P2(2)], 'LineWidth',2,'Color','red');
    
    line8 = line([P5(1) P6(1)],[P5(2) P6(2)] , 'LineWidth',3 ,'Color','red');
    line9 = line([P6(1) P7(1)],[P6(2) P7(2)], 'LineWidth',2,'Color','red');
    
    line5 = line([P9(1) P10(1)],[P9(2) P10(2)], 'LineWidth',3,'Color','red');
    line6 = line([P10(1) P11(1)],[P10(2) P11(2)], 'LineWidth',2,'Color','red');
    
    line10 = line([P13(1) P14(1)],[P13(2) P14(2)] , 'LineWidth',3 ,'Color','red');
    line11 = line([P14(1) P15(1)],[P14(2) P15(2)], 'LineWidth',2,'Color','red');
    
    line15 = line([P1(1) P5(1)],[P1(2) P5(2)], 'LineWidth',5);
    
    %vel = inv(J)*[-sin(t)*a; cos(t)*b];
    %Update Interval;
    pause(0.01);
    %remove previous identifiers
    delete(line2);
    delete(line3);
    %delete(line4);
    delete(line5);
    delete(line6);
    delete(line8);
    delete(line9);
    delete(line10);
    delete(line11);
    delete(line15);
    %delete(Traj_identi1);
end