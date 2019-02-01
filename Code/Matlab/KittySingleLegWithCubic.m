%Link Lengths
l1 = 26; l2 = 22;

%Trajectory Parameters
a = 20; b = 7;

axis(gca, 'equal');
axis([-50 50 -50 50]);
grid on;

w = 0.1;
%loop
for t=0:1:100000
    syms t;
    %End effector speed & Coordinates
    xe = cos(w*t)*a;
    ye = sin(w*t)*b -40;
    
    %Future coordinates
    xef = cos(w*(t+1))*a;
    yef = sin(w*(t+1))*b -40;
    
    syms t1;
    syms t2;
    syms t1f;
    syms t2f;
    
    %Trajectory center coordinates
    P4 = [0,-40];
    
    %HipCoordinate
    P1 = [0,0];
    
    %End Effector Trajectory (ellipse)
    if(ye< -40)
        ye= -40;
    end
    
    %End effector coordinates
    P3D = [xe ye];
    Desired_Traj_identi = viscircles(P3D,0.01);
    
    %End Effector current coord.
    xa = l1*cos(t1) + l2*cos(t1 + t2);
    ya = l1*sin(t1) + l2*sin(t1 + t2);
    %End Effector future coord.
    xaf = l1*cos(t1f) + l2*cos(t1f + t2f);
    yaf = l1*sin(t1f) + l2*sin(t1f + t2f);
    
    %Joint velocities Jacobian
    J = jacobian([xa,ya], [t1, t2]);
    Jf = jacobian([xaf,yaf], [t1f, t2f]);
    
    %Calculating Joint angles current values
    if(atan(ye/xe)>0)
        alpha = atan(ye/xe)-pi;
    else
        alpha = atan(ye/xe);
    end
    
    theta1 = cosine_rule(l2,l1,sqrt(xe^2+ye^2))+alpha;
    theta2 = -pi+cosine_rule(sqrt(xe^2+ye^2),l1,l2);
    
    %Calculating Joint angles future values
    if(yef< -40)
        yef= -40;
    end
    if(atan(yef/xef)>0)
        alpha = atan(yef/xef)-pi;
    else
        alpha = atan(yef/xef);
    end
    theta1f = cosine_rule(l2,l1,sqrt(xef^2+yef^2))+alpha;
    theta2f = -pi+cosine_rule(sqrt(xef^2+yef^2),l1,l2);
    
    %Jacobian current with values
    Jv = subs(J, {t1 , t2}, {theta1, theta2});
    %Jacobian future with values
    Jvf = subs(Jf, {t1f , t2f}, {theta1f, theta2f});
    
    %Joints Velocity Current
    X = inv(Jv)*[-(sin(w*t))*a*w ; (cos(w*t))*b*w];
    
    %Joints Velocity future
    Xf = inv(Jvf)*[-(sin(w*(t+1)))*a*w ; (cos(w*(t+1)))*b*w];
    
    %Desired
    %Cubic
    for ti = t:t+1
    t0 = t;
    tf = t+1;
    M = [ 1 t0 t0^2 t0^3;
      0 1 2*t0 3*t0^2;
      1 tf tf^2 tf^3;
      0 1 2*tf 3*tf^2];
  
    %Cubic Joint1
    co1 = [theta1; X(1); theta1f; Xf(1)];

    c1 = inv(M)*co1

    qd1 = c1(1) + c1(2).*ti + c1(3).*ti.^2 + c1(4).*ti.^3;
    vd1 = c1(2) + 2*c1(3).*ti + 3*c1(4).*ti.^2;
    ad1 = 2*c1(3) + 6*c1(4).*ti;
    
    %Cubic Joint2
    co2 = [theta2; X(2); theta2f; Xf(2)];

    c2 = inv(M)*co2;

    qd2 = c2(1) + c2(2).*ti + c2(3).*ti.^2 + c2(4).*ti.^3;
    vd2 = c2(2) +2*c2(3).*ti + 3*c2(4).*ti.^2;
    ad2 = 2*c2(3) + 6*c2(4).*ti;
    
    %Forward Kinematics
    xd1 = double(l1*cos(qd1));
    yd1 = double(l1*sin(qd1));
    xd2 = double(l1*cos(qd1) + l2*cos(qd1 + qd2));
    yd2 = double(l1*sin(qd1) + l2*sin(qd1 + qd2));
    if(yd2 < -40)
        yd2 = -40;
    end
    %Current Knee Coordinates
    P2C = [xd1 , yd1];
    %Current End Effector Coordinates
    P3C = [xd2 , yd2];
    %End effector coordinates identifier
    %Actual_Traj_identi = viscircles(P3C,0.01);
    
    %Actual Leg trajectory display
    line5 = line([P1(1) P2C(1)],[P1(2) P2C(2)]);
    line6 = line([P3C(1) P2C(1)],[P3C(2) P2C(2)]);
        %line7 = line([P4(1) P2C(1)],[P4(2) P2C(2)]);
    
    pause(0.01);
        
    delete(line5);
    delete(line6);
    
    end
    
    % Desired Knee Coordinates
    P2D = [l1*cos(theta1) , l1*sin(theta1)];
    
    %Desired Leg trajectory display
    line2 = line([P1(1) P2D(1)],[P1(2) P2D(2)]);
    line3 = line([P3D(1) P2D(1)],[P3D(2) P2D(2)]);
        %line4 = line([P4(1) P2D(1)],[P4(2) P2D(2)]);
        
    
    
    
        %vel = inv(J)*[-sin(t)*a; cos(t)*b];
    
    %Update Interval;
%     pause(0.01);
    
    %Remove Previous Identifiers
    delete(line2);
    delete(line3);

        %delete(line4);
        %delete(line7);
        %delete(Desired_Traj_identi);
        %delete(Actual_Traj_identi);
end