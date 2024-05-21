function [pwcurve] = genPWCurve(waypoints,time)
% % SECTION 3 - GENERATE BEZIER PIECES
%preallocate array of curves


N_dim = size(waypoints,2);


pwcurve = cell(1,N_dim);
% for each dimension
for n_dim = 1:N_dim
    N_wpts = size(waypoints{n_dim},1);
    N_der = size(waypoints{n_dim},2);
    pwcurve{n_dim} = cell(N_wpts-1,1);
    % for each section (between two waypoints)
    for n = 1:(N_wpts-1)
        % select relevant waypoints
        wpts = [waypoints{n_dim}(n,:); waypoints{n_dim}(n+1,:)];
        % select relevant time period
        if iscell(time)
            tint = time{n_dim}(n+(0:1));
        else
            tint = time(n+(0:1));
        end
        % interpolate between two points
        Q = cell(1,N_der);
        tctl = cell(1,N_der);
        Q{1} = interpHermBern(wpts,tint);
        tctl{1}    = linspace(tint(1),tint(2),numel(Q{1}));
        for n_der = 1:N_der-1
            Q{1+n_der} = diffBernControl(Q{1},tint,n_der);
            tctl{1+n_der}    = linspace(tint(1),tint(2),numel(Q{1+n_der}));
        end
        
        out.ctl     = Q;
        out.tint    = tint;
        out.tctl    = tctl;
        pwcurve{n_dim}{n} = out;
    end
end
end














