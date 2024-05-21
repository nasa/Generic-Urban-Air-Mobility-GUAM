function Q = intBernControl(t,P,n,varargin)
%INT_BEZIER Q = int_bezier(t,P,n,varargin)
%           time interval, control points, number of integrations,
%           initialcondition
P = P(:);
N = size(P,1);
Ndeg = N-1+n;

z = zeros(n,1); % deg elevation
if n>1
    disp('n>1 TO BE IMPLEMENTED') %recursive function calls n->0
    return;
end
Q = factorial(Ndeg-n) / factorial(Ndeg) * (t(end)-t(1)).^n * [z;cumsum(P,n)];
if ~isempty(varargin)
    IC = varargin{1};IC = IC(:);
else
    IC = z;
end
Q = Q+IC;
end

% function Q = int_bezier(t,P,n,varargin)
% %INT_BEZIER Q = int_bezier(t,P,n,varargin)
% %           time interval, control points, number of integrations,
% %           initialcondition
% P = P(:);
% N = size(P,1);
% Ndeg = N-1+n;
% 
% z = zeros(n,1); % deg elevation
% if n>1
%     disp('n>1 TO BE IMPLEMENTED') %recursive function calls n->0
%     return;
% end
% Q = factorial(Ndeg-n) / factorial(Ndeg) * (t(end)-t(1)).^n * [z;cumsum(P,n)];
% if ~isempty(varargin)
%     IC = varargin{1};IC = IC(:);
% else
%     IC = z;
% end
% Q = Q+IC;
% end
