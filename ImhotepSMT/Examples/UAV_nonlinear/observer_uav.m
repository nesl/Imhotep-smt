function [shat, obsevStatus] = observer_uav(sensorIndex, measurements, inputs)

    global tstep cstep;

    shat        = zeros(13,1);
    obsevStatus = 1;
%     if(sum(ismember([1 2 3 7 8 9], sensorIndex)) < 6 )
%         obsevStatus = -1;
%         return;
%     end
    
    params = nanoplus();
    Rot         = RPYtoRot_ZXY(measurements{7}(2),measurements{8}(2), measurements{9}(2));
    Rot         = Rot';
    Ax          = [0 1; 0 0]; Bx = [0; (1/params.mass)*Rot(1,3)];
    sysCx       = ss(Ax,Bx,[0 1], 0);
    sysDx       = c2d(sysCx, cstep);
    
    x_minus     = measurements{1}(1);
    x           = measurements{1}(2);
    %vx_hat      = (x - x_minus)/cstep;
%   vx_hat      = vx_hat/5;
     vx_hat      = x - x_minus*sysDx.A(1,1) - sysDx.B(1,1)*inputs{1}(2);
     vx_hat      = vx_hat/sysDx.A(1,2);
    
    
    y_minus     = measurements{2}(1);
    y           = measurements{2}(2);
    vy_hat      = (y - y_minus)/cstep; 
    %vy_hat      = vy_hat/5;
    
    z_minus     = measurements{3}(1);
    z           = measurements{3}(2);
    vz_hat      = (z - z_minus)/cstep;
    %vz_hat      = vz_hat/5;
    
    phi_minus   = measurements{7}(1);
    phi         = measurements{7}(2);
    wPhi_hat    = (phi - phi_minus)/cstep;
    
    theta_minus = measurements{8}(1);
    theta       = measurements{8}(2);
    wTheta_hat  = (theta - theta_minus)/cstep;

    yaw_minus   = measurements{9}(1);
    yaw         = measurements{9}(2);
    wYaw_hat    = (yaw - yaw_minus)/tstep;  
    
    wPhi_hat    = measurements{10}(2);
    wTheta_hat  = measurements{11}(2);
    wYaw_hat    = measurements{12}(2);
    
%     if(ismember(4, sensorIndex))
%         vx_hat      = measurements{4}(2);
%     else
%         x_minus     = measurements{1}(1);
%         x           = measurements{1}(2);
%         vx_hat      = (x - x_minus)/tstep;
%     end
    
%     if(ismember(5, sensorIndex))
%         vy_hat      = measurements{5}(2);
%     else
%         y_minus     = measurements{2}(1);
%         y           = measurements{2}(2);
%         vy_hat      = (y - y_minus)/tstep;
%     end    
%     
%     if(ismember(6, sensorIndex))
%         vz_hat      = measurements{6}(2);
%     else
%         z_minus     = measurements{3}(1);
%         z           = measurements{3}(2);
%         vz_hat      = (z - z_minus)/tstep;
%     end
    
    
%     if(ismember(10, sensorIndex))
%         wPhi_hat    = measurements{10}(2);
%     else
%         phi_minus   = measurements{7}(1);
%         phi         = measurements{7}(2);
%         wPhi_hat    = (phi - phi_minus)/tstep;
%     end
%     
%     if(ismember(11, sensorIndex))
%         wTheta_hat  = measurements{11}(2);
%     else
%         theta_minus = measurements{8}(1);
%         theta       = measurements{8}(2);
%         wTheta_hat  = (theta - theta_minus)/tstep;
%     end
    
%     
%     if(ismember(12, sensorIndex))
%         wYaw_hat    = measurements{12}(2);
%     else
%         yaw_minus   = measurements{9}(1);
%         yaw         = measurements{9}(2);
%         wYaw_hat    = (yaw - yaw_minus)/tstep;
%     end
    
    
    % x,y,z
    shat(1)     = measurements{1}(2);
    shat(2)     = measurements{2}(2);
    shat(3)     = measurements{3}(2);
    
    % vx, vy, vz
    shat(4)     = vx_hat;
    shat(5)     = vy_hat;
    shat(6)     = vz_hat;
    
    % phi, theta, yaw
    shat(7:10)  = RotToQuat(RPYtoRot_ZXY(measurements{7}(2),measurements{8}(2), measurements{9}(2)));
    
    % omega_phi, omega_theta, omega_yaw
    shat(11)    = wPhi_hat;
    shat(12)    = wTheta_hat;
    shat(13)    = wYaw_hat;
    
end