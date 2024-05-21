function [values] = evalPWCurve(pwcurve,times,order)
% %

times = times(:)';

N_dim = size(pwcurve,2);
N_times = size(times,1);

values = zeros(N_times,N_dim);
% for each dimension
for n_dim = 1:N_dim
    % pull out time intervals
    intervals = cell2mat(arrayfun(@(e) e.tint(:)',[pwcurve{n_dim}{:}],'UniformOutput',false)');
    % compute edges defined required by discretize
%     intervals = [intervals(1:end-1)' intervals(2:end)'];%
    edges = [intervals(1:2) intervals(2:end,2)']';
    
    % lookup indices of times (hopefully something clever [O(n)] underneath)
    int_idx = discretize(times,edges);
    % evaluate bezier curves at each time
    N_edge = length(edges);
    for n_bin = 1:(N_edge-1)
        % construct time values
        idx = n_bin==int_idx;
        t = times(idx);
        % scale time in interval
        tint = intervals(n_bin,:);
        % find values
        Q = pwcurve{n_dim}{n_bin}.ctl{order+1};
        values(idx,n_dim) = evalBernPoly(Q,t(:),tint);
    end
end

end

