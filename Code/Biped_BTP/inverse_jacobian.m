function [ theta_1dot, theta_2dot, theta_1ddot, theta_2ddot] = inverse_jacobian( x_dot, y_dot, x_ddot, y_ddot, theta_1, theta_2, l )
J= [ -l(1)*sin(theta_1)-l(2)*sin(theta_1+theta_2), -l(2)*sin(theta_1+theta_2)
     -l(1)*cos(theta_1)-l(2)*cos(theta_1+theta_2), -l(2)*cos(theta_1+theta_2)];
  velocity = [x_dot y_dot]';
  theta_dot = inv(J)*velocity;
  theta_1dot = theta_dot(1);
  theta_2dot = theta_dot(2);  
  J_dot = [ -l(1)*cos(theta_1)*theta_1dot-l(2)*cos(theta_1+theta_2)*(theta_1dot+theta_2dot), -l(2)*cos(theta_1+theta_2)*(theta_1dot+theta_2dot)
            l(1)*sin(theta_1)*theta_1dot+l(2)*sin(theta_1+theta_2)*(theta_1dot+theta_2dot), l(2)*sin(theta_1+theta_2)*(theta_1dot+theta_2dot)];
 
  theta_ddot= inv(J)*([x_ddot, y_ddot]' - J_dot*theta_dot);
  theta_1ddot =theta_ddot(1);
  theta_2ddot = theta_ddot(2);  

end

