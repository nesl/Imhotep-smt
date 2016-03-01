function [xhat, obsevStatus] = observer_ugv(sensorIndex, measurements, inputs)
global Ts;

    xhat        = zeros(5,1);
    obsevStatus = 1;
    if(sum(ismember([1 2], sensorIndex)) < 2 )
        obsevStatus = -1;
        return;
    end
    
    
    x_minus     = measurements{1}(1);
    x           = measurements{1}(2);
    x_plus      = measurements{1}(3);
    
    
    
    y_minus     = measurements{2}(1);
    y           = measurements{2}(2);
    y_plus      = measurements{2}(3);
    
    
    theta_hat       = atan2((y - y_minus),(x - x_minus));
    theta_hat_plus  = atan2((y_plus - y),( x_plus - x));
    
    v_hat       = (x - x_minus)/(Ts*cos(theta_hat));
    w_hat       = (theta_hat_plus - theta_hat)/Ts;
    
    xhat(1)     = v_hat;
    xhat(2)     = w_hat;
    xhat(3)     = x_plus;
    xhat(4)     = y_plus;
    xhat(5)     = theta_hat_plus;
    
%     for counter = 1 : length(sensorIndex)
%         sensorIndex(counter)
%         measurements{sensorIndex(counter)}
%     end
%     xhat
end