function [u, theta] = Limited_Long_Cont_Alloc(M_lon, mdes_lon, eng_max, eng_min, act_max, act_min)

% Compute the single matrix longitudinal allocation
u_act = M_lon*mdes_lon;

scale_factor = 1; % Initialize scale factor to 1

% Check if any engines exceed max position limit
if any(u_act(1:9) > eng_max)
    for loop = 1:9
        scale_temp = eng_max(loop)/(M_lon(loop,:)*mdes_lon);
        if scale_temp>0 && scale_temp<1 && scale_temp < scale_factor
            scale_factor = scale_temp;
        end
    end
end
% Check if elevator exceed limits
if u_act(10)>act_max(3)
    scale_temp = act_max(3)/(M_lon(10,:)*mdes_lon);
    if scale_temp>0 && scale_temp<1 && scale_temp<scale_factor
        scale_factor = scale_temp;
    end
end
if u_act(10)<act_min(3)
    scale_temp = act_min(3)/(M_lon(10,:)*mdes_lon);
    if scale_temp>0 && scale_temp<1 && scale_temp<scale_factor
        scale_factor = scale_temp;
    end
end
% Check if elevator or flaps exceed limits
if u_act(11)>act_max(1)
    scale_temp = act_max(1)/(M_lon(11,:)*mdes_lon);
    if scale_temp>0 && scale_temp<1 && scale_temp<scale_factor
        scale_factor = scale_temp;
    end
end
if u_act(11)<act_min(1)
    scale_temp = act_min(1)/(M_lon(11,:)*mdes_lon);
    if scale_temp>0 && scale_temp<1 && scale_temp<scale_factor
        scale_factor = scale_temp;
    end
end

% Scale allocation if required
if scale_factor ~= 1
    u_act = M_lon*mdes_lon*scale_factor;
end

% Parse out the effector allocations
u       = u_act(1:end-1);
theta   = u_act(end);