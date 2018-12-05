function [theta] = cosine_rule(c,b,a)  
cos_theta=(a^2+b^2-c^2)/(2*a*b);
theta = acos(cos_theta);