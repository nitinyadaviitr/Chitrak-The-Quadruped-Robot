function [ a1, a2 ] = trajectory_generator( theta1_o, theta1_f, theta2_o, theta2_f, tf)

a1(1) = theta1_o(1);
a1(2) = theta1_o(2);
a1(3) = theta1_o(3)/2;
a1(4) = (20*(theta1_f(1) -theta1_o(1)) -(8*theta1_f(2) + 12*theta1_o(2))*tf -(3*theta1_o(3)-theta1_f(3))*tf^2)/(2*tf^3);
a1(5) = (30*(theta1_o(1) -theta1_f(1)) +(14*theta1_f(2) + 16*theta1_o(2))*tf +(3*theta1_o(3)-2*theta1_f(3))*tf^2)/(2*tf^4);
a1(6) = (12*(theta1_f(1) -theta1_o(1)) -6*(theta1_f(2) + theta1_o(2))*tf -(theta1_o(3) -theta1_f(3))*tf^2)/(2*tf^5);

a2(1) = theta2_o(1);
a2(2) = theta2_o(2);
a2(3) = theta2_o(3)/2;
a2(4) = (20*(theta2_f(1) -theta2_o(1)) -(8*theta2_f(2) + 12*theta2_o(2))*tf -(3*theta2_o(3)-theta2_f(3))*tf^2)/(2*tf^3);
a2(5) = (30*(theta2_o(1) -theta2_f(1)) +(14*theta2_f(2) + 16*theta2_o(2))*tf +(3*theta2_o(3)-2*theta2_f(3))*tf^2)/(2*tf^4);
a2(6) = (12*(theta2_f(1) -theta2_o(1)) -6*(theta2_f(2) + theta2_o(2))*tf -(theta2_o(3) -theta2_f(3))*tf^2)/(2*tf^5);


end

