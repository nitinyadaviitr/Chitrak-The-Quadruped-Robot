function [ thet1, thet2] = inverse_kinematics_stance(x, y, a1, a2 )
num = (x^2 +y^2 - (a1)^2 - (a2)^2);
den = 2*a1*a2;
thet_2 = acos(num/den);

sin_1 = ((a2 + a1*cos(thet_2))*y - a1*sin(thet_2)*x)/( x^2 + y^2);
cos_1 = ((a2 + a1*cos(thet_2))*x + a1*sin(thet_2)*y)/( x^2 + y^2);
thet_1 = atan2(sin_1,cos_1);

% thet1= pi-thet_1 -thet_2;
thet1= pi-thet_1 -thet_2;
thet2= thet_2;


end

