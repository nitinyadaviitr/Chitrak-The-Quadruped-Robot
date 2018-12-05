function [ thet1, thet2] = inverse_kinematics_stance(x, y, l)
num = (x^2 +y^2 - (l(1))^2 - (l(2))^2);
den = 2*l(1)*l(2);
thet_2 = acos(num/den);

sin_1 = ((l(2) + l(1)*cos(thet_2))*y - l(1)*sin(thet_2)*x)/( x^2 + y^2);
cos_1 = ((l(2) + l(1)*cos(thet_2))*x + l(1)*sin(thet_2)*y)/( x^2 + y^2);
thet_1 = atan2(sin_1,cos_1);

thet1= pi-thet_1 -thet_2;
thet2=thet_2;

end
