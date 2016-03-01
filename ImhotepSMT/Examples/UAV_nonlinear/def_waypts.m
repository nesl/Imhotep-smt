function waypts = def_waypts()
% start points used to simulate takeoff
start_pt = [0 0 0;
            0 0 1];

% defined waypts here
%**************** define waypts ***************************** 
waypts = [2.5 0 1;
          2.5 2.5 1;
          0 2.5 1;
          0 0 1]; % square
      
%******************* end ************************************

waypts = [start_pt;
          waypts];

end

