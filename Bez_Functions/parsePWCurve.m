% demo_5_genPWCurve.m
%   demonstrate piecewise curve
clearvars
clc
addpath('../../Functions/Bezier')
cmap = lines;

N_wpts = [20 20 20];
N_der = [3 3 10];
dt = 1/100;
for n_dim = 1:3
    waypoints{n_dim} = randn(N_wpts(n_dim),1);%N_der(n_dim)
    time_wpts{n_dim} = [0,cumsum(round(3*sqrt(randn(1,N_wpts(n_dim)-1).^2 + randn(1,N_wpts(n_dim)-1).^2)/dt)*dt)];
end
for n_dim = 1:3
    for m = 1:N_der(n_dim)-1
        waypoints{n_dim}(:,1+m) = gradient(waypoints{n_dim}(:,m))./gradient(time_wpts{n_dim}');
    end
end

T = max(cell2mat(time_wpts));
time = 0:dt:T;
tic
pwcurve = genPWCurve(waypoints,time_wpts);
toc
tic
values = evalPWCurve(pwcurve,time,0);
toc

% f(1)=figure(1);clf(f(1));
% for ndim = 1:3
%     ab(ndim)=subplot(3,1,ndim);grid('on');hold('on');
% end
% for ndim = 1:3
%     for m = 1:3
%         plot(ab(m),time_wpts{ndim},waypoints{ndim}(:,m),'s','MarkerSize',5,'Color',cmap(ndim,:),'MarkerFaceColor',cmap(ndim,:));
%     end
%     for nseg = 1:length(pwcurve{ndim})
%         for m = 1:3
%             plot(ab(m),pwcurve{ndim}{nseg}.tctl{m},pwcurve{ndim}{nseg}.ctl{m},'.','MarkerSize',15,'Color',cmap(ndim,:));
%         end
%     end
% end
% linkaxes(ab,'x');
% ylabel(ab(1),'${x}$');
% ylabel(ab(2),'$\dot{x}$');
% ylabel(ab(3),'$\ddot{x}$');
% xlabel('t');
% for m = 1:3
%     v = evalPWCurve(pwcurve,time,m-1);
%     plot(ab(m),time,v(:,1),'Color',cmap(1,:));
%     plot(ab(m),time,v(:,2),'Color',cmap(2,:));
%     plot(ab(m),time,v(:,3),'Color',cmap(3,:));
% end

%%%%%%%%%%%%%%%%%%%% CREATE VECTORS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% number of dimensions
N_dim = length(pwcurve);
% number of segments per dimension
N_segs = cellfun(@(x) length(x),pwcurve);
N_der = nan(1,sum(N_segs));
N_ctl = nan(1,sum(N_segs));
for n_dim = 1:N_dim
    for n_segs = 1:N_segs(n_dim)
        % add after all previous segments
        ind = sum(N_segs(1:n_dim-1)) + n_segs;
        % number of derivatives per segment
        N_der(ind) = length(pwcurve{n_dim}{n_segs}.ctl);
        N_ctl(ind) = length(pwcurve{n_dim}{n_segs}.ctl{1});
    end
end
%%%%%% Output 1
parseVect = [N_dim,N_segs,N_ctl,N_der];
%%%%%% Output 2
TVECT = nan(sum(N_segs),2);
%%%%%% Output 3
CVECT = nan(sum(N_ctl),1);

for n_dim = 1:N_dim
    for n_segs = 1:N_segs(n_dim)
        ind = sum(N_segs(1:n_dim-1)) + n_segs;
        TVECT(ind,:) = pwcurve{n_dim}{n_segs}.tint;
        
        % add after all previous segments' control points
        ctl_idx = sum(N_ctl(1:ind-1)) + (1 : N_ctl(ind));
        CVECT(ctl_idx) = pwcurve{n_dim}{n_segs}.ctl{1};
%         
%         M = N_der(ind);
%         for m = 1:M
%             cell2mat(pwcurve{n_dim}{n_segs}.ctl')
%         end
    end
end



%%%%%%%%%%%%%%%%%%%% PARSE VECTORS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
idxDim = 1;
idxEnd = length(parseVect);
N_dim2 = parseVect(idxDim);
idxSegs = 2:(2+N_dim2-1);
N_segs2 = parseVect(idxSegs);
idxCtl = (idxSegs(end)+1):(idxSegs(end)+sum(N_segs2));
N_ctl2 = parseVect(idxCtl);
idxDer = idxCtl(end)+1:idxEnd;
N_der2 = parseVect(idxDer);

for n_dim = 1:N_dim2
    for n_segs = 1:N_segs2(n_dim)

    end
end
