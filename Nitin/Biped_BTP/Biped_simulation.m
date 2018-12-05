%Thigh and leg length
clc
clear all
l= [0.7 0.55 0.7 0.55];
lc= [0.2 0.1 0.2 0.1];
m= [2 1 2 1]; 
Ic= [2 1 2 1];
P0= [0 , 0];
P1= [0 , 0]; P2= [0 , 0]; P4= [0 , 0]; P5= [ 0, 0];
theta = [ 1.200 , 0.467, 1.531 , 0.467];
theta_dot = [0 0 0 0];
theta_ddot = [0 0 0 0];
err= [0 0 0 0];
err_dot= [0 0 0 0];
err_sum= [0 0 0 0];

K= [ 16 8 1];
axis(gca, 'equal');
axis([-2 3 -2 2.5]);
grid on;

%COM parameters
z = 1.2;
g = 9.81;
Tc = sqrt(z/g);
origin =[0 0];
stride_length =0.4;
x_dot0 = 2;
x_0 = -stride_length/2;
h = 0.2;
stride_time = Tc*log((-stride_length/2 -Tc*x_dot0)/(stride_length/2 -Tc*x_dot0));
i=0;
flag=1;
dt= 0.01;

%swing leg tajectories
Po = [-x_0, z]; Pm= [0, z-h]; Pf=[x_0, z];
Po_dot = [-x_dot0, 0]; Pm_dot = [0.5, 0]; Pf_dot = [-x_dot0, 0];
Po_ddot = [-g*x_0/z, 0]; Pm_ddot = [0.5, 0]; Pf_ddot = [-g*x_0/z, 0];

[theta_1o, theta_2o] = inverse_kinematics_stance( Po(1), Po(2), l );
[theta_1odot, theta_2odot, theta_1oddot, theta_2oddot] = inverse_jacobian( Po_dot(1), Po_dot(2), Po_ddot(1), Po_ddot(2), theta_1o, theta_2o, l);
[theta_1m, theta_2m] = inverse_kinematics_stance( Pm(1), Pm(2), l);
[theta_1mdot, theta_2mdot, theta_1mddot, theta_2mddot] = inverse_jacobian( Pm_dot(1), Pm_dot(2), Pm_ddot(1), Pm_ddot(2), theta_1m, theta_2m, l);
[theta_1f, theta_2f] = inverse_kinematics_stance( Pf(1), Pf(2), l);
[theta_1fdot, theta_2fdot, theta_1fddot, theta_2fddot] = inverse_jacobian( Pf_dot(1), Pf_dot(2), Pf_ddot(1), Pf_ddot(2), theta_1f, theta_2f, l);

theta_1o=[theta_1o, theta_1odot, theta_1oddot];
theta_1m=[theta_1m, theta_1mdot, theta_1mddot];
theta_1f=[theta_1f, theta_1fdot, theta_1fddot];
theta_2o=[theta_2o, theta_2odot, theta_2oddot];
theta_2m=[theta_2m, theta_2mdot, theta_2mddot];
theta_2f=[theta_2f, theta_2fdot, theta_2fddot];

for step= 1:1:6
  
for t=0:dt:10
i =i+1;
%stance leg trajectory
x(i)= x_0*cosh(t/Tc) + Tc*x_dot0*sinh(t/Tc);
x_dot(i) = x_0*sinh(t/Tc)/Tc +x_dot0*cosh(t/Tc);
x_ddot(i) = g/z*x(i);
[thetaD(1), thetaD(2)] = inverse_kinematics_stance(x(i), z, l);
[thetaD_dot(1), thetaD_dot(2), thetaD_ddot(1), thetaD_ddot(2)] = inverse_jacobian(-x_dot(i), 0,-x_ddot(i), 0, thetaD(1), thetaD(2),l);

%swing leg trajectory
if (t< stride_time/2)
[a3, a4] = trajectory_generator(theta_1o, theta_1m, theta_2o, theta_2m, stride_time/2);
thetaD(3) = a3(1) + a3(2)*t +a3(3)*t^2 + a3(4)*t^3 + a3(5)*t^4 +a3(6)*t^5;
thetaD(4) = a4(1) + a4(2)*t +a4(3)*t^2 + a4(4)*t^3 + a4(5)*t^4 +a4(6)*t^5;

thetaD_dot(3) = a3(2) +2*a3(3)*t + 3*a3(4)*t^2 + 4*a3(5)*t^3 +5*a3(6)*t^4;
thetaD_dot(4) = a4(2) +2*a4(3)*t + 3*a4(4)*t^2 + 4*a4(5)*t^3 +5*a4(6)*t^4;

thetaD_ddot(3) = 2*a3(3) + 6*a3(4)*t + 12*a3(5)*t^2 +20*a3(6)*t^3;
thetaD_ddot(4) = 2*a4(3) + 6*a4(4)*t + 12*a4(5)*t^2 +20*a4(6)*t^3;

else
[a3, a4] = trajectory_generator(theta_1m, theta_1f, theta_2m, theta_2f, stride_time/2);
t1= t- stride_time/2;
thetaD(3) = a3(1) + a3(2)*t1 +a3(3)*t1^2 + a3(4)*t1^3 + a3(5)*t1^4 +a3(6)*t1^5;
thetaD(4) = a4(1) + a4(2)*t1 +a4(3)*t1^2 + a4(4)*t1^3 + a4(5)*t1^4 +a4(6)*t1^5;

thetaD_dot(3) = a3(2) +2*a3(3)*t1 + 3*a3(4)*t1^2 + 4*a3(5)*t1^3 +5*a3(6)*t1^4;
thetaD_dot(4) = a4(2) +2*a4(3)*t1 + 3*a4(4)*t1^2 + 4*a4(5)*t1^3 +5*a4(6)*t1^4;

thetaD_ddot(3) = 2*a3(3) + 6*a3(4)*t1 + 12*a3(5)*t1^2 +20*a3(6)*t1^3;
thetaD_ddot(4) = 2*a4(3) + 6*a4(4)*t1 + 12*a4(5)*t1^2 +20*a4(6)*t1^3;
end

%Inverse Dynamics and and controller
com_dot(1) =x_dot(i); com_dot(2) = x_ddot(i);
[B_q, C_q_qdot, G_q, D_x, E_x] = inverse_dynamics( theta, theta_dot, l, lc, com_dot, m, Ic );
F_ext= G_q + D_x*com_dot(2) + E_x*com_dot(1);
[ err, err_dot, err_sum ] = calculate_error( thetaD, thetaD_dot, theta, theta_dot, err);
[Torque_dash, Torque]= controller( B_q, C_q_qdot, F_ext, err, err_dot, err_sum, thetaD_ddot, theta_dot, K);

%Calculating Thetas
theta_ddot= Torque_dash;
theta_dot = theta_dot + theta_ddot*dt;
theta= theta + theta_dot*dt;

%forward kinematics
if (flag ==1)
P0 = [origin(1)+x(i) z];
P1 = P0 + [l(1)*cos(thetaD(1)) , -l(1)*sin(thetaD(1))];
P2 = P0 + [l(1)*cos(thetaD(1)) + l(2)*cos(thetaD(1)+thetaD(2)) , -l(1)*sin(thetaD(1)) - l(2)*sin(thetaD(1) + thetaD(2))];
P4 = P0 + [l(1)*cos(thetaD(3)) , -l(1)*sin(thetaD(3))];
P5 = P0 + [l(1)*cos(thetaD(3)) + l(2)*cos(thetaD(3)+thetaD(4)) , -l(1)*sin(thetaD(3))-l(2)*sin(thetaD(3) + thetaD(4))];
simulation( P0, P1, P2, P4, P5, origin, x(i), z );
if(x(i)>stride_length/2)
    flag =0;
     origin = origin + [stride_length 0];
     i=0;
     break;
end 
else
P0 = [origin(1)+x(i) z];
P4 =  P0 + [l(1)*cos(thetaD(1)) , -l(1)*sin(thetaD(1))];
P5 = P0 +[l(1)*cos(thetaD(1)) + l(2)*cos(thetaD(1)+thetaD(2)) , -l(1)*sin(thetaD(1)) - l(2)*sin(thetaD(1) + thetaD(2))];
P1 = P0 + [l(1)*cos(thetaD(3)) , -l(1)*sin(thetaD(3))];
P2 = P0 + [l(1)*cos(thetaD(3)) + l(2)*cos(thetaD(3)+thetaD(4)) , -l(1)*sin(thetaD(3))-l(2)*sin(thetaD(3) + thetaD(4))];
simulation( P0, P1, P2, P4, P5, origin, x(i), z );
if(x(i)>stride_length/2)
     origin = origin + [stride_length 0];
     i=0;
    flag = 1;
    break; 
end
end
%plot velocities and acceleration
% theta_3(i)= thetaD(3);
% theta_4(i)= thetaD(4);
% theta_3(21+i) = thetaD(1);
% theta_4(21+i) = thetaD(2);
% theta_1(i)= thetaD(1);
% theta_2(i)= thetaD(2);
% theta_1(21+i) = thetaD(3);
% theta_2(21+i) = thetaD(4);
% time(i)=t;
% time(21+i) = 0.21 + t;
% thet_dot3(i) = thetaD_dot(3);
% thet_dot4(i) = thetaD_dot(4);
% thet_dot3(21+i) = thetaD_dot(1);
% thet_dot4(21+i) = thetaD_dot(2);
% thet_dot1(i) = thetaD_dot(1);
% thet_dot2(i) = thetaD_dot(2);
% thet_dot1(21+i) = thetaD_dot(3);
% thet_dot2(21+i) = thetaD_dot(4);
% err_1(i) = err(1);
% err_2(i) = err(2);
% err_1(21+i) = err(1);
% err_2(21+i) = err(2);
% err_3(i) = err(3);
% err_4(i) = err(4);
% err_3(21+i) = err(1);
% err_4(21+i) = err(2);

end

end
%  plot (time, theta_3, 'blue');
%  hold
%  plot (time, theta_4, 'red');
%   subplot(2,2,1);  
%   plot(time, err_1);
%   title('Subplot 1: err_1');
%   ylabel('error in radian'); 
%   xlabel('time in seconds') ;
%   subplot(2,2,2);
%   plot(time, err_2);
%   title('Subplot 1: err_2');
%   ylabel('error in radian'); 
%   xlabel('time in seconds') ;
%   subplot(2,2,3);
%   plot(time, err_3);
%   title('Subplot 1: err_3');
%   ylabel('error in radian'); 
%   xlabel('time in seconds') ;
%   subplot(2,2,4);
%   plot(time, err_4);
%   title('Subplot 1: err_4');
%   ylabel('error in radian'); 
%   xlabel('time in seconds') ;
