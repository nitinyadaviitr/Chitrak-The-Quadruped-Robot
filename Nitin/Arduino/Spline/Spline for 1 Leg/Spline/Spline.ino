void setup() {
  // put your setup code here, to run once:

}

void loop() {
    Jv = subs(J, {t1 , t2}, {theta1, theta2});
    //Jacobian future with values
    Jvf = subs(Jf, {t1f , t2f}, {theta1f, theta2f});
    
    //Joints Velocity Current
    X = inv(Jv)*[-(sin(w*t))*a*w ; (cos(w*t))*b*w];
    
    //Joints Velocity future
    Xf = inv(Jvf)*[-(sin(w*(t+1)))*a*w ; (cos(w*(t+1)))*b*w];
    
    //Desired
    //Cubic
    for ti = t:t+1
    t0 = t;
    tf = t+1;
    M = [ 1 t0 t0^2 t0^3;
      0 1 2*t0 3*t0^2;
      1 tf tf^2 tf^3;
      0 1 2*tf 3*tf^2];
  
    //Cubic Joint1
    co1 = [theta1; X(1); theta1f; Xf(1)];

    c1 = inv(M)*co1

    qd1 = c1(1) + c1(2).*ti + c1(3).*ti.^2 + c1(4).*ti.^3;
    vd1 = c1(2) + 2*c1(3).*ti + 3*c1(4).*ti.^2;
    ad1 = 2*c1(3) + 6*c1(4).*ti;
    
    //Cubic Joint2
    co2 = [theta2; X(2); theta2f; Xf(2)];

    c2 = inv(M)*co2;

    qd2 = c2(1) + c2(2).*ti + c2(3).*ti.^2 + c2(4).*ti.^3;
    vd2 = c2(2) +2*c2(3).*ti + 3*c2(4).*ti.^2;
    ad2 = 2*c2(3) + 6*c2(4).*ti;
    
    //Forward Kinematics
    xd1 = double(l1*cos(qd1));
    yd1 = double(l1*sin(qd1));
    xd2 = double(l1*cos(qd1) + l2*cos(qd1 + qd2));
    yd2 = double(l1*sin(qd1) + l2*sin(qd1 + qd2));
    if(yd2 < -40)
        yd2 = -40;
    end

}
