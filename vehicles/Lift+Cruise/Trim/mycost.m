%function [cost, g] = mycost(x,x0,FreeVar, Q, a, offset_x0, scale, diff_eng_wt)
% function [cost] = mycost(x,x0,FreeVar, Q, a, offset_x0, scale, diff_eng_wt)
function [cost, g] = mycost(x, X0, FreeVar, Q, offset_x0, scale, gang_vec)


% state and input cost for trimming
x0              = zeros(length(FreeVar),1);
x0(~FreeVar)    = X0(~FreeVar);
x0(FreeVar)     = x;

% Get boolean index of FreeVar + Ganged Vars
% TuneVars = FreeVar | gang_vec'~=0;

% Update the gang vector as appropriate (assumes same scaling for ganged effectors)
uniq_gang = unique(gang_vec);
uniq_gang = uniq_gang(uniq_gang~=0); % Remove the zeros
if isempty(uniq_gang)
    % Now compute the cost
    %x0              = x0.*scale;
    %cost = (x0.*scale-offset_x0.*scale)'*Q*(x0.*scale-offset_x0.*scale);

    % Note only use FreeVar for cost (other portions are fixed...)
    cost = (x0(FreeVar).*scale(FreeVar)-offset_x0(FreeVar).*scale(FreeVar))'*Q(FreeVar,FreeVar)*(x0(FreeVar).*scale(FreeVar)-offset_x0(FreeVar).*scale(FreeVar));
else
    % Cycle thru each unique ganged surface and set each ganged to first
    % instance of the ganged surfaces
    disp('')
    for surf = 1:length(uniq_gang)
        sur_ind = find(gang_vec==uniq_gang(surf));
        x0(sur_ind) = x0(sur_ind(1));
    end
    % Scale x0 now...
    %x0              = x0.*scale;
    % Now compute the cost
    % cost = (x0.*scale-offset_x0.*scale)'*Q*(x0.*scale-offset_x0.*scale);
        
    % Note only use FreeVar for cost (other portions are fixed...)
    cost = (x0(FreeVar).*scale(FreeVar)-offset_x0(FreeVar).*scale(FreeVar))'*Q(FreeVar,FreeVar)*(x0(FreeVar).*scale(FreeVar)-offset_x0(FreeVar).*scale(FreeVar));
end

if nargout > 1 % gradient required
    % D_leading  = [ 4*(x(9)-x(10))  -4*(x(9)-2*x(10)+x(11))  -4*(x(10)-2*x(11)+x(12)) -4*(x(11)-x(12)) ];
    % D_trailing = [ 4*(x(13)-x(14)) -4*(x(13)-2*x(14)+x(15)) -4*(x(14)-2*x(15)+x(16)) -4*(x(15)-x(16)) ];
    % g = 2*x0'*Q + a + [ zeros(1,8) D_leading D_trailing 0 ];

    % Note only use FreeVar for cost (other portions are fixed...)
    g = 2*(x0(FreeVar).*scale(FreeVar)-offset_x0(FreeVar).*scale(FreeVar))'*Q(FreeVar,FreeVar)*diag(scale(FreeVar));

    % Note only use TuneVars for cost (other portions are fixed...)
    % g = 2*(x0(TuneVars).*scale(TuneVars)-offset_x0(TuneVars).*scale(TuneVars))'*Q(TuneVars,TuneVars)*diag(scale(TuneVars));
    
    %     if ~isempty(uniq_gang)
    %         % However, to get agreement with numerical gradients, need to use
    %         % "multiplicity" factor for ganged effectors (since ganged effectors
    %         for surf = 1:length(uniq_gang)
    %             sur_ind         = find(gang_vec==uniq_gang(surf));
    %             mult_factor     = sum(sur_ind~=0);
    %             g(sur_ind(1))   = g(sur_ind(1))*mult_factor;
    %         end
    %     end

    % Use all variables (despite some being fixed to X0)
    % g = 2*(x0.*scale-offset_x0.*scale)'*Q*diag(scale);
    % g = g(FreeVar);
    
    disp('');
      % if nargout > 2 % Hessian required
      %     H = [1200*x(1)^2-400*x(2)+2, -400*x(1);
      %         -400*x(1), 200];  
      % end

end
