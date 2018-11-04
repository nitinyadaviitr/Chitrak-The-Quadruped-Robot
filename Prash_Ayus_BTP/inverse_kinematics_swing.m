function [ thet3, thet4 ] = inverse_kinematics_swing( x, y, a3, a4 )
num = (x^2 +y^2 - (a3)^2 - (a4)^2);
den = 2*a3*a4;
cos_2 = num/den;
sin_2 =-sqrt(1-cos_2^2);
%thet4 = atan2(sin_2,cos_2);
thet4 = acos(num/den);
% if (thet4 <0)
%     thet4 = acos(num/den);
% else
%     thet4 = -acos(num/den);
% end   
sin_1 = ((a3 + a4*cos(thet4))*y - a4*sin(thet4)*x)/( x^2 + y^2);
cos_1 = ((a3 + a4*cos(thet4))*x + a4*sin(thet4)*y)/( x^2 + y^2);
thet3 = atan2(sin_1,cos_1);
% if (thet3 <0)
%     thet3 = atan2(sin_1,cos_1);
% else
%     thet3 = -atan2(sin_1,cos_1);
% end    
end

