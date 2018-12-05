function [ err, err_dot, err_sum ] = calculate_error( thetaD, thetaD_dot, theta, theta_dot, prev_err )
err= thetaD - theta;
err_dot= thetaD_dot - theta_dot;
err_sum= prev_err + err;

end

