function [B_q, C_q_qdot, G_q, D_x, E_x] = inverse_dynamics( theta, theta_dot, l, lc, com_dot, m, Ic )
%short forms for cosine and sine
g= 9.81;

c1 = cos(theta(1));
c2 = cos(theta(2));
c12 = cos(theta(1)+theta(2));
c3 = cos(theta(3));
c4 = cos(theta(4));
c34 = cos(theta(3)+theta(4));

s1 = sin(theta(1));
s2 = sin(theta(2));
s12 = sin(theta(1)+theta(2));
s3 = sin(theta(3));
s4 = sin(theta(4));
s34 = sin(theta(3)+theta(4));

%parameters for B matrix
A1 = (m(1) +m(2))/2;
A2 = (Ic(1) + Ic(2) + m(1)*lc(1)^2 + m(2)*l(2)^2 + m(2)*lc(2)^2)/2 + m(2)*l(1)*lc(2)*c2;
A3 = (Ic(2) + m(2)*lc(2)^2)/2;

B1 = -(m(1)*lc(1)*s1 + m(2)*l(1)*s1 + m(2)*lc(2)*s12);
B2 = (Ic(2) + m(2)*lc(2)^2 + m(2)*l(1)*lc(2)*c2);
B3 = -m(2)*lc(2)*s12;

C1 = (m(3) +m(4))/2;
C2 = (Ic(3) + Ic(4) + m(3)*lc(3)^2 + m(4)*l(4)^2 + m(4)*lc(4)^2)/2 + m(4)*l(3)*lc(4)*c4;
C3 = (Ic(4) + m(4)*lc(4)^2)/2;

D1 = -(m(3)*lc(3)*s3 + m(4)*l(3)*s3 + m(4)*lc(4)*s34);
D2 = (Ic(4) + m(4)*lc(4)^2 + m(4)*l(3)*lc(4)*c4);
D3 = -m(4)*lc(4)*s34;

%parameters for C matrix
dB1_dtheta1 = -m(1)*lc(1)*c1 - m(2)*l(1)*c1 - m(2)*lc(2)*c12;
dB3_dtheta1 = -m(2)*l(2)*c12;

dA2_dtheta2 = -m(2)*l(1)*lc(2)*s2;
dB1_dtheta2 = -m(2)*lc(2)*c12;
dB2_dtheta2 = -m(2)*l(1)*lc(2)*s2;
dB3_dtheta2 = -m(2)*l(2)*c12;

dD1_dtheta3 = -(m(3)*lc(3)*c3 + m(4)*l(3)*c3 + m(4)*lc(4)*c34);
dD3_dtheta3 = -m(4)*l(4)*c34;

dC2_dtheta4 = -m(4)*l(3)*lc(4)*s4;
dD1_dtheta4 = -m(4)*lc(4)*c34;
dD2_dtheta4 = -m(4)*l(3)*lc(4)*s4;
dD3_dtheta4 = -m(4)*l(4)*c34;


dA1_dt = 0 ;
dA2_dt = -m(2)*l(1)*lc(2)*s2*theta_dot(2);
dA3_dt = 0;

dB1_dt = -(theta_dot(1)*(m(1)*lc(1)*c1 + m(2)*l(1)*c1 + m(2)*lc(2)*c12) + theta_dot(2)*(m(2)*lc(2)*c12));
dB2_dt = -m(2)*l(1)*lc(2)*s2*theta_dot(2);
dB3_dt = -m(2)*l(2)*c12*(theta_dot(1)+theta_dot(2));

dC1_dt = 0;
dC2_dt = -m(4)*l(3)*lc(4)*s4*theta_dot(4);
dC3_dt = 0;

dD1_dt = -(theta_dot(3)*(m(3)*lc(3)*c3 + m(4)*l(3)*c3 + m(4)*lc(4)*c34) + theta_dot(4)*(m(4)*lc(4)*c34));
dD2_dt = -m(4)*l(3)*lc(4)*s4*theta_dot(4);
dD3_dt = -m(4)*l(4)*c34*(theta_dot(3) + theta_dot(4));

%computation of the matrices
dT_dt_dthetaDot = [2*dA2_dt, dB2_dt, 0, 0;
                dB2_dt, 2*dA3_dt, 0, 0;
                0, 0, 2*dC2_dt, dD2_dt;
                0, 0, dD2_dt, 2*dC3_dt;];
v11= com_dot(1)*dB1_dtheta1;
v12= com_dot(1)*dB3_dtheta1;
v21= theta_dot(1)*dA2_dtheta2 +com_dot(1)*dB1_dtheta2;
v22= theta_dot(1)*dB2_dtheta2 +com_dot(1)*dB3_dtheta2;
v33= com_dot(1)*dD1_dtheta3;
v34= com_dot(1)*dD3_dtheta3;
v43= theta_dot(3)*dC2_dtheta4 +com_dot(1)*dD1_dtheta4;
v44= theta_dot(3)*dD2_dtheta4 +com_dot(1)*dD3_dtheta4;

dT_dtheta = [v11, v12, 0, 0;
             v21, v22, 0, 0;
             0, 0, v33, v34;
             0, 0, v43, v44;];

B_q= [ 2*A2, B2, 0, 0;
       B2, 2*A3, 0, 0;
       0, 0, 2*C2, D2;
       0, 0, D2, 2*C3;];

D_x= [ B1 B3 D1 D3]';


C_q_qdot = dT_dt_dthetaDot -dT_dtheta;


E_x = [ dB1_dt dB3_dt dD1_dt dD3_dt]';

G_q = [ -g*(m(1)*lc(1)*c1 + m(2)*l(1)*c1 + m(2)*lc(2)*c12);
        -g*m(2)*lc(2)*c12;
        -g*(m(1)*lc(3)*c3 + m(2)*l(3)*c3 + m(2)*lc(4)*c34);
        -g*m(2)*lc(4)*c34;];
           
end

