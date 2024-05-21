%close all; % Close open figures..

% using optimization to solve for trim conditions with constraints
tiltwing = build_Lift_plus_Cruise();
tiltwing.om_p = [0 0 0 0 0 0 0 0 0];

% *************************************************************************
global POLY
POLY = 1; % Denotes to use poly database
blending_method = 2; % Polynomial blending method 
% ************ BLENDING METHOD MUST MATCH WHAT IS USED IN SIM!!! **********

simSetup; % Need to perform this to get access to SimIn quantities
ft2kts = 1/(SimIn.Units.nmile/SimIn.Units.ft/3600); % Obtained from setUnits, Conversion factor of ft/sec to knots
kts2ft = (SimIn.Units.nmile/SimIn.Units.ft/3600); % Obtained from setUnits,  Conversion factor from knots to ft/sec
a = 1125.33; % Speed of sound ft/sec

% *************************************************************************
% out_path        = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver2p0'; % Trim_Ver1p0
%out_path        = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver2p0'; % Trim_Ver2p0
%out_path        = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver3p0'; % Trim_Ver3p0
out_path        = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver4p0'; % Trim_Ver3p0
save_flag       = 0;

% Specifies the trim case to run...
% *************************************************************************
% UH = [0:10:130]*kts2ft; % Trim_Ver1p0 
%UH = [0:5:130]*kts2ft; % Trim_Ver2p0 
%UH = [0:5:90 94.8 94.9 100:5:130]*kts2ft; %Trim_Ver3p0x
%UH = [0:5:90 94.8 95 100:5:130]*kts2ft; %Trim_Ver3p0x
UH = [0:5:90 94.8 95 100:5:130]*kts2ft; % Trim_Ver4p0 

WH = 700/60; % 'Trim_Designs_Long_Descend';
R = [inf]; % R = [2500 5000 inf];
case_name       = 'Trim_Designs_Long_descend'; 
% trim_des_num  = 6.1; % Trim_Ver1p0 
% trim_des_num  = 6.2; % Trim_Ver2p0 
% trim_des_num    = 6.3; % Trim_Ver3p0 
trim_des_num    = 6.4; % Trim_Ver4p0 
% *************************************************************************

% *************************************************************************
% % UH = [0:10:130]*kts2ft; %[0:10:130]*kts2ft; %Trim_Ver1p0
% % UH =  [0:10:130]*kts2ft; %Trim_Ver2p0x
% % UH = [0:5:90 94.8 94.9 100:5:130]*kts2ft; %Trim_Ver3p0x
% % UH = [0:5:90 94.8 95 100:5:130]*kts2ft; %Trim_Ver3p0x
% % UH = [0:5:80 81:1:90 94.8 95 100:10:130]*kts2ft; % Trim_Ver4p0 test refinement
% UH = [0:5:90 94.8 95 100:5:130]*kts2ft; % Trim_Ver4p0 test refinement
% %UH = [0:5:90 94.8 95 100]*kts2ft;
% %UH = [95 100:5:130]*kts2ft;
% 
% WH = -450/60; % 'Trim_Designs_Long_Climb'; % Trim_Ver3p0
% R = [inf]; % R = [2500 5000 inf];
% case_name       = 'Trim_Designs_Long_climb';
% % trim_des_num    = -6.1; %Trim_Ver1p0
% % trim_des_num    = -6.2; %Trim_Ver2p0
% %trim_des_num    = -6.3; %Trim_Ver3p0
% %trim_des_num    = -6.4; %Trim_Ver4p0
% trim_des_num    = -6.5; %Trim_Ver4p0


% *************************************************************************

% *************************************************************************
% % UH = [0:10:130]*kts2ft; %Trim_Ver1p0
% % UH = [0:5:130]*kts2ft; %Trim_Ver2p0  
% %UH = [0:5:90 94.8 94.9 100:5:130]*kts2ft; % Trim_Ver3p0
% % UH = [0:5:90 94.8 95 100:5:130]*kts2ft; % Trim_Ver3p0
% %UH = [50:5:75 80 81:1:90 94.8 95 100]*kts2ft; % Trim_Ver3p0 test refinement
% UH = [0:5:90 94.8 95 100:5:130]*kts2ft; % Trim_Ver4p0 test refinement
% WH = 0/60; % 'Trim_Designs_Long';
% R = [inf]; % R = [2500 5000 inf];
% case_name       = 'Trim_Designs_Long'; 
% % trim_des_num    = 6; % 'Trim_Designs_Long'; % Case 6 is final version (Trim_Ver1p0)
% %trim_des_num    = 6.2; % 'Trim_Designs_Long'; % Case 6.1 is experimental, 6.2 is version for Trim_ver2p0 
% %trim_des_num    = 6.3; % 'Trim_Designs_Long'; % Case 6.1 is experimental, 6.3 is version for Trim_ver3p0 
% trim_des_num    = 6.4; % 'Trim_Designs_Long'; % Case6.4 is version to fix continuous manifold issues for Trim_ver3p0 
% *************************************************************************

% *************************************************************************
% UH = [0:10:130]*kts2ft;[0:10:130]*kts2ft; %[0:10:130]*kts2ft; 
% WH = 0/60; % 'Trim_Designs_Long';
% R = [2500]; % R = [2500 5000 inf];
% case_name       = 'Trim_Designs_Turn'; 
% trim_des_num    = 1; % 'Trim_Designs_Long';
% *************************************************************************

% *************************************************************************
% UH = [0:10:130]*kts2ft;[0:10:130]*kts2ft; %[0:10:130]*kts2ft; 
% WH = 0/60; % 'Trim_Designs_Long';
% R = [2500]; % R = [2500 5000 inf];
% case_name       = 'Trim_Designs_Turn'; 
% trim_des_num    = 1; % 'Trim_Designs_Long';
% *************************************************************************

trans_start     = (50*kts2ft);
% trans_end       = (95*kts2ft); % Trim_Ver2p0
% trans_end       = (90*kts2ft); %Trim_Ver3p0
trans_end       = (94.8*kts2ft); %Trim_Ver4p0

max_evals       = 5000; % Set max fmincon function evals
max_iterations  = 500; % Set max fmincon iterations
max_evals2      = 11000; % Set alternate max fmincon function evals
max_iterations2 = 1000; % Set alternate max fmincon iterations

LPC = tiltwing;

GRAV    = SimIn.Environment.Earth.Gravity.g0(3);
RHO     = SimIn.Environment.Atmos.rho0;

% Provide Numbers for configure
Ns              = 4; % Number of surfaces
Np              = 9; % Number of propellers
Num_FreeVars    = 18; 
gang_vec        = [];

if POLY
  % Bounds for polynomial model
%   LB = [-12*pi/180  -60*pi/180 -2*pi -2*pi -2*pi -30*pi/180 -30*pi/180 -30*pi/180 -30*pi/180 repmat(58.5959,1,4)   repmat(58.5959,1,4)    79.000 ];
%   UB = [ 12*pi/180   60*pi/180  2*pi  2*pi  2*pi  30*pi/180   0*pi/180  30*pi/180 30*pi/180 repmat(162.3156,1,4)  repmat(162.3156,1,4)  183.25   ];
%         th         phi       p      q     r     flp       ail        elv        rud         rl          rt         pp 
  LB = [-12*pi/180  -60*pi/180 -2*pi -2*pi -2*pi -30*pi/180 -30*pi/180 -30*pi/180 -30*pi/180 repmat(0,1,4)   repmat(0,1,4)    0  ];
  UB = [ 12*pi/180   60*pi/180  2*pi  2*pi  2*pi  0*pi/180   30*pi/180  30*pi/180 30*pi/180 repmat(162.3156,1,4)  repmat(162.3156,1,4)  183.25   ];

else
  % Bounds for strip theory
  LB = [ -5*pi/180  -60*pi/180 -2*pi -2*pi -2*pi -30*pi/180 -30*pi/180 -30*pi/180 -30*pi/180 repmat(0,1,4)    repmat(0,1,4)     0        ];
  UB = [ 15*pi/180   60*pi/180  2*pi  2*pi  2*pi  30*pi/180  30*pi/180  30*pi/180  30*pi/180 repmat(250,1,4)  repmat(250,1,4)   400.00   ];
end

% Bookkeeping and data storage
XEQ = nan(length(UH), Num_FreeVars+3, length(WH), length(R));
XEQ_All = nan(length(UH),Num_FreeVars+3, length(WH), length(R));
exitcount = 0;
exitflag = []; % initialize exit flag to zero...
output_store = [];
% Loop'd loop
for ii = 1:length(R)
  for kk = 1:length(WH)
    count = 0;
    for jj = 1:length(UH)
        count = count +1;
        if UH(jj) <= trans_start 
            % Hover
            run(case_name); % Trim_Designs_Long;
            FreeVar         = FreeVar_hover;
            offset_x0       = offset_x0_hover;
            x0              = X0_hover(FreeVar);% reset the intial to hover; 
            X0              = X0_hover;
            scale           = scale_hover;
            gang_vec        = gang_hover;
            Q               = Q_hover;
        elseif UH(jj) > trans_start && UH(jj) <= trans_end 
            % Transition
            run(case_name); % Trim_Designs_Long;
            FreeVar             = FreeVar_trans;
            offset_x0           = offset_x0_trans;
            x0                  = X0_trans(FreeVar); % reset the intial to cruise; 
            X0                  = X0_trans;
            scale               = scale_trans;
            gang_vec            = gang_trans;
            Q                   = Q_trans;
        else 
            % Cruise
            run(case_name); % Trim_Designs_Long;
            FreeVar             = FreeVar_cruise;
            offset_x0           = offset_x0_cruise;
            x0                  = X0_cruise(FreeVar); % reset the intial to cruise; 
            X0                  = X0_cruise;
            scale               = scale_cruise;
            gang_vec            = gang_cruise;
            Q                   = Q_cruise;
        end
      TRIM_POINT = [UH(jj) WH(kk) R(ii)];

      % Check compatability between FreeVar and gang_vec: Are any FreeVars
      % also ganged (except for first instance)? If so, throw a fault
      [uniq_gang1, gang_ind]     = unique(gang_vec);
      uniq_gang  = uniq_gang1(uniq_gang1~=0);   
      gang_ind = gang_ind(uniq_gang1~=0); % gets the indices of the first non-zero instance of each gang integer
      gang_ind_bool = true(Num_FreeVars,1); % Initialize boolean to all ones.. 
      gang_ind_bool(gang_ind) = 0; % Sets the boolean for each first non-zero instance to zero..
      if any((gang_vec' & FreeVar) & gang_ind_bool')
          disp('One or More FreeVar variables are selected and are also ganged (other than first instance).');
          disp('Deselect the FreeVar for each ganged effector (except the first instance)');
          fault_ind = find((gang_vec' & FreeVar) & gang_ind_bool');
          fprintf('\nBased on the gang choices, the FreeVars to deselect are:');
          fprintf('Index: #%i\t',fault_ind);
          fprintf('Exiting....\n');
          return
      end

      % Set the desired lower and upper bounds...
      lb = LB(FreeVar);
      ub = UB(FreeVar);
        
      % use fmincon to solve a constrained optimization propblem
      opts = optimoptions(@fmincon,'algorithm','interior-point','tolcon', 1e-7, ... % 'interior-point' 'active-set', 'sqp'
          'Display', 'final', 'SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient', false, ...
          'MaxFunctionEvaluations', max_evals,'CheckGradients', false,...
          'FiniteDifferenceType','central', 'MaxIterations', max_iterations, 'StepTolerance', 1e-10);
      opts2 = optimoptions(@fmincon,'algorithm','interior-point','tolcon', 1e-7, ... % 'interior-point' 'active-set', 'sqp'
          'Display', 'final', 'SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient', false, ...
          'MaxFunctionEvaluations', max_evals2,'CheckGradients', false,...
          'FiniteDifferenceType','central', 'MaxIterations', max_iterations2, 'StepTolerance', 1e-10);
                          %"EnableFeasibilityMode",true,"SubproblemAlgorithm","cg");
                          %'Display', 'final',
      disp(' ********************************* Running fmincon *********************************')
      [ x, fval, exitflag, output, lambda, grad, hessian ] = fmincon(@(x) mycost(x, X0, FreeVar, Q, offset_x0, scale, gang_vec),...
          x0,[],[],[],[],lb,ub, @(x) nlinCon_helix(x,[TRIM_POINT X0(~FreeVar)'],LPC,GRAV,RHO,Ns,Np,FreeVar, gang_vec, kts2ft, a, blending_method),opts);
      disp(' ********************************* fmincon complete*********************************')

      if exitflag <= 0
          if ~isempty(output.bestfeasible)
              disp('*************** Running alternate fmincon using bestfeasible found.. ****************');
              X02 = X0;
              X02(FreeVar) = output.bestfeasible.x;
            [ x, fval, exitflag, output, lambda, grad, hessian ] = fmincon(@(x) mycost(x, X02, FreeVar, Q, offset_x0, scale, gang_vec),...
               x0,[],[],[],[],lb,ub, @(x) nlinCon_helix(x,[TRIM_POINT X0(~FreeVar)'],LPC,GRAV,RHO,Ns,Np,FreeVar, gang_vec, kts2ft, a, blending_method),opts2);
            disp('*************** Completed alternate fmincon using bestfeasible ****************');
            disp('');
            if exitflag <= 0
                if ~isempty(output.bestfeasible.x)
                    disp('*************** No feasible solution found after running alternate fmincon..Outputing last feasible solution found.');
                    % Just output the best feasible
                    exitflag = 99;
                    x = output.bestfeasible.x;
                end
            end
             
          else
            disp('******************* Running fmincon again with opts2... *********************************');
            [ x, fval, exitflag, output, lambda, grad, hessian ] = fmincon(@(x) mycost(x, X0, FreeVar, Q, offset_x0, scale, gang_vec),...
            x0,[],[],[],[],lb,ub, @(x) nlinCon_helix(x,[TRIM_POINT X0(~FreeVar)'],LPC,GRAV,RHO,Ns,Np,FreeVar, gang_vec, kts2ft, a, blending_method),opts2);
            disp('******************* Completed fmincon again with opts2... *********************************');

            if exitflag <= 0
                if ~isempty(output.bestfeasible)
                    disp('*************** No feasible solution found after running alternate fmincon..Outputing last feasible solution found.');
                    % Just output the best feasible
                    exitflag = 99;
                    x = output.bestfeasible.x;
                else
                    disp('*************** No feasible solution found ... Outputing the infeasible result.'); 
                end
            end
          end
      end

% [cost, g] = mycost(x, X0, FreeVar, Q, offset_x0, scale, gang_vec)
[c, ceq]= nlinCon_helix(x,[TRIM_POINT X0(~FreeVar)'],LPC,GRAV,RHO,Ns,Np,FreeVar, gang_vec, kts2ft, a, blending_method)

output_store{count} = output;
      % update the current equilibrium vector
      xeq = X0;
      xeq(FreeVar) = x;

      % Now update the final output vector due to gang_vec as appropriate..
      if any(gang_vec)
          % First find the unique (non-zero gang vector entries)
        uniq_gang = unique(gang_vec);
        uniq_gang = uniq_gang(uniq_gang ~= 0); 
        % Now cycle thru them and substitute the values
        for gang_loop = uniq_gang'
            g_ind = find(gang_vec==gang_loop);
            for inner = 1:length(g_ind)
              xeq(g_ind(inner)) = xeq(g_ind(1));
            end
        end
      end

      % th phi p q r flp ail elv rud rl rt pp
      fprintf('----------------------\n');
      fprintf('trim point: Ground Speed = %0.2f kts, Vertical Speed = %0.2f ft/min, R = %0.2f ft\n\n',...
              TRIM_POINT.*[ft2kts -60 1]);
      fprintf('  theta = %0.4f deg\n', xeq(1)*180/pi);
      fprintf('  phi   = %0.4f deg\n', xeq(2)*180/pi);
      fprintf('  om    = [%0.4f %0.4f %0.4f ]deg/sec\n', xeq(3:5)*180/pi);
      fprintf('  delf  = %0.4f deg\n', xeq(6)*180/pi);
      fprintf('  dela  = %0.4f deg\n', xeq(7)*180/pi);
      fprintf('  dele  = %0.4f deg\n', xeq(8)*180/pi);
      fprintf('  delr  = %0.4f deg\n', xeq(9)*180/pi);
      fprintf('  om_l  = [%0.4f %0.4f %0.4f% 0.4f] rad/s\n', xeq(10:13));
      fprintf('  om_t  = [%0.4f %0.4f %0.4f %0.4f] rad/s\n', xeq(14:17));
      fprintf('  om_p  = %0.4f rad/s\n', xeq(18));

      fprintf('\n  dchi  = %0.4f deg/s\n', UH(jj)/R(ii)*180/pi);
      fprintf('\n  Cost = %0.4e\n\n', fval);

      % check exit flag before saving point
      if exitflag > 0

        fprintf('  exitflag = %u\n',exitflag);
        fprintf('-- trim point saved --\n');
        fprintf('----------------------\n');

        %if ~isempty(output.bestfeasible) && exitflag ~= 99 && any(output.bestfeasible.x-x)
        if exitflag ~= 99 && ~isempty(output.bestfeasible) &&  any(output.bestfeasible.x-x)...
                && ((fval > output.bestfeasible.fval) && output.bestfeasible.constrviolation <1e-6)
            % Logic here: 1) not exitflag 99 (already handled below),
            % 2) A bestfeasible different from x exists
            % 3) Best feasible has a lower cost AND meets the constraint
            % violation (<1e-6)
            
          % update the current equilibrium vector using best feasible
          xeq_alt = X0;
          xeq_alt(FreeVar) = output.bestfeasible.x;
    
          % Now update the final output vector due to gang_vec as appropriate..
          if any(gang_vec)
              % First find the unique (non-zero gang vector entries)
            uniq_gang = unique(gang_vec);
            uniq_gang = uniq_gang(uniq_gang ~= 0); 
            % Now cycle thru them and substitute the values
            for gang_loop = uniq_gang'
                g_ind = find(gang_vec==gang_loop);
                for inner = 1:length(g_ind)
                  xeq_alt(g_ind(inner)) = xeq(g_ind(1));
                end
            end
          end
          XEQ(jj,:,kk,ii) = [ TRIM_POINT xeq' ];
          x0 = x;
          exitcount = 0;
          disp(' ******************* Best Feasible stored in XEQ_ALL *****************');
          % Store all results regardless of convergence status
          XEQ_All(jj,:,kk,ii) = [ TRIM_POINT xeq_alt' ];
          disp('')

        else
            % NOTE: cases here include: 
            % Case 99: no feasible solution was returned, just return best
            % feasible (which was set to x previously)
            XEQ(jj,:,kk,ii) = [ TRIM_POINT xeq' ];
            x0 = x;
            exitcount = 0;
            
            % Store all results regardless of convergence status
            XEQ_All(jj,:,kk,ii) = [ TRIM_POINT xeq' ];
        end

      else 
        % Negative flag results (no solution and no best feasible)
        % Store all results regardless of convergence status          
        XEQ_All(jj,:,kk,ii) = [ TRIM_POINT xeq' ];

        exitcount = exitcount+1;
        % if exitcount > 5, break, end % 

      end

    end % UH

    XEQi = XEQ(:,:,kk,ii);
    show_trim(XEQi,1);
    sgtitle(sprintf('Trim Case: %0.3g,  acc = %0.3g, wh = %0.3g, R = %0.3g',trim_des_num, 0,WH(kk), R(ii)));
    drawnow
    han = gcf;
    han.Name = sprintf('Trim_Case_%0.3g_R%0.3g_WH%0.3g',trim_des_num, R(ii), WH(kk));
    % Now show all cases regardless of convergence...
    XEQi_All = XEQ_All(:,:,kk,ii);
    show_trim(XEQi_All,1);
    sgtitle(sprintf('Trim All Case: %0.3g,  acc = %0.3g, wh = %0.3g, R = %0.0f',trim_des_num, 0,WH(kk), R(ii)));
    drawnow
    han = gcf;
    han.Name = sprintf('Trim_All_Case_%0.3g_R%0.3g_WH%0.3g',trim_des_num, R(ii), WH(kk));
    
    % Save the results if flagged to do so...
    if save_flag
        Multi_Fig_Save_func(out_path, 1.4, 1.4,'.fig','.png');
        save(fullfile(out_path, sprintf('Trim_Case_%0.3g_R%0.3g_WH%0.3g_XEQ.mat', trim_des_num, R(ii), WH(kk))), "XEQ","XEQ_All","UH", "WH", "R",...
            "trans_start", "trans_end", "FreeVar_hover","offset_x0_hover","scale_hover", "gang_hover", ...
            "FreeVar_trans", "offset_x0_trans", "scale_trans", "gang_trans", ...
            "FreeVar_cruise", "offset_x0_cruise", "scale_cruise", "gang_cruise",'-v7.3');
        disp('Results Saved!!!!!');
    else
        disp("Results NOT SAVED!!!!");
    end
  end % R

end % WH 

% Plot some results


