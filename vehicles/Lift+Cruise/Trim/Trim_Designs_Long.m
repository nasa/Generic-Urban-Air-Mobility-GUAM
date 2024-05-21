% Trim_Designs_Long: This file accepts a trim design number and from that
% details the particulars of a trim design used by trim_helix.m  
% In particular, the inputs and outputs for this file are: 
%
% Input: 
% trim_des_num: Specifies case number to run
% 
% Output:
% Q:            Quadratic cost matrix 
% ********** The following variables come in three varieties by appending
%           one of three suffixes: _hover, _trans or _cruise
% X0:           Initial conditions vector
% FreeVars:     Vector of variables optimizer can search
% scale:        Scaling vector (used to normalize state vector)
% gang_vec:     Vector used to gang effectors (effectors with same integer are ganged) 
%
% Written by:
% Mike Acheson, Dynamic Systems and Control Branch (DSCB) D-316
% NASA Langley Research Center (LaRC), 6/12/2023
% 
% Modifications: Date/Initials/Details
% *************************************************************************

% *************************************************************************
% Set quadratic cost component Q
q_theta = 1; % pitch angle
q_phi   = 1 ; % roll angle 
q_p     = 1 ; % roll rate
q_q     = 1 ; % pitch rate
q_r     = 1 ; % yaw rate
q_del_f = 1; % flap deflection
q_del_a = 1 ; % aileron deflection
q_del_e = 1; % elevator deflection
q_del_r = 1 ; % rudder deflection
q_om_l  = 1*[1 1 1 1] ; % leading edge prop speed 
q_om_t  = 1*[1 1 1 1]; % trailing edge prop speed 
q_om_p  = 1; % pusher prop speed
% *************************************************************************

switch trim_des_num
    case 1 % Just Raw input...
        
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th         phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover = [0*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 0];
        %                            th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover   =  boolean([ 1   1   0   0   0  1   1   1   1   [1 1 1 1]   [1 1 1 1]   0  ]);
        % Prescribe the offset vector for the cost computation
        offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
        % Normalized scaling...
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';        
        gang_hover          = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        
        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   1   0   0   0  1   1   1   1   [1 1 1 1]   [1 1 1 1]   1  ]);
        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [4*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  1   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

    case 2 % Gang leading rotors in hover and transition, gang trailing rotors in hover and transition..
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th         phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover = [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 0];
        %                            th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover   =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 0 0]   [1 0 0 0]   0  ]);
        offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
        % Normalized scaling...
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_hover          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [0*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(75,4,1); repmat(75,4,1); 90];
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 0 0]   [1 0 0 0]   1  ]);
        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set Cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [4*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 125];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  1   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

    case 3
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th         phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover = [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 0];
        %                            th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover   =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   0  ]);

        % Prescribe the offset vector for the cost computation
        offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
        
        % Normalized scaling...
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';         
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [0*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(75,4,1); repmat(75,4,1); 90];
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   0  ]);

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [4*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 125];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  1   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

    case 4 % Prescribe flap schedule
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th         phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover = [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 0];
        %                            th  phi p   q   r  flp ail elv rud rl          rt         pp 
        %FreeVar_hover   =  boolean([ 1   1   0   0   0  1   0   0   1   [1 0 0 0]   [1 0 0 0]   1  ]);
        FreeVar_hover   =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
%          offset_x0_hover(1)  = UH(jj)/trans_start *5*pi/180; % Prescribe pitch to 10 in 2 deg
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
%          offset_x0_hover(10:17) = 90*ones(8,1); % Prescribe lifting rotor nominal
%          offset_x0_hover(18) = 125;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 

        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
                    
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
       % gang_hover          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        %gang_hover          = [zeros(9,1); 1*ones(8,1);0];

        %gang_hover          = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        
        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [5*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 125];
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   1   0   0   0  0   1   1   1   [1 0 0 0]   [1 0 0 0]   1  ]);

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
%          offset_x0_trans(1)  = 6*pi/180; % Prescribe pitch to 10
%         if UH(jj) > (trans_end - 10)
%             offset_x0_hover(10:17) = 60*ones(8,1)*(trans_end-UH(jj))/10; % Prescribe lifting rotor nominal
%         end

        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        %gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);


        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [4*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        % scale_cruise        = [180/pi 1000*180/pi 180/pi 180/pi 180/pi 180/pi 10000*180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        % Non-normalized scaling...
        % scale             = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]';
         gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
    case 5.4 % Prescribe flap schedule AND pitch angle schedule (Cases 5.1-5.4 various gangings)
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        %X0_hover = [-2*pi/180;  0*pi/180; 0; 0; 0; (1-UH(jj)/trans_start)*-25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];

        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        %FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 1 1 1]   [1 1 1 1]   1  ]);
        %FreeVar_hover    =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 0 0]   [1 0 0 0]   1  ]);
        % FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);
         FreeVar_hover     =  boolean([ 1   1   0   0   0  1   1   1   1   [1 1 0 1]   [0 1 0 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180; % Prescribe pitch to 10 in 2 deg increments

         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; % Prescribe lifting rotor nominal
%          offset_x0_hover([10,14],1) = 120 -60*UH(jj)/trans_start;  % Prescribe lifting rotor nominal
%          offset_x0_hover([13,17],1) = 120 -60*UH(jj)/trans_start;  % Prescribe lifting rotor nominal
         offset_x0_hover(18) = 100; % 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th        phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
                    
        % gang_hover          = zeros(18,1);
        %gang_hover          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        % gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
         gang_hover          = [zeros(9,1); 1; 2; 2; 3; 1; 4; 4; 3; 0];

        
        % gang_hover          = [zeros(9,1); 1*ones(8,1);0];

        % gang_hover          = zeros(18,1);

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        % Q_hover(1,10) = 100;Q_hover(1,12) = 100;
        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  1   1   1   1   [1 0 0 0]   [1 0 0 0]   1  ]);

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        % offset_x0_trans(1)  = 10*pi/180; % Prescribe pitch to 10
        if UH(jj) > (trans_end - 10)
            offset_x0_hover(10:17) = 60*ones(8,1)*(trans_end-UH(jj))/10; % Prescribe lifting rotor nominal
        end

        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];
        %gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [4*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        % scale_cruise        = [180/pi 1000*180/pi 180/pi 180/pi 180/pi 180/pi 10000*180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        % Non-normalized scaling...
        % scale             = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

    case 6 % This is the schedule that works.  Case 6.1 is experimental
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];

        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180;  % Prescribe pitch to 6 in 2 deg increments
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; 60; 95; % Prescribe lifting rotor nominal
         offset_x0_hover(18) =  100; 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';                 
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
%X0_trans = [9*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(15,4,1); repmat(0,4,1); 110];

        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  0   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        offset_x0_trans(1)  = 6*pi/180+UH(jj)/trans_end *1.8*pi/180;  % Prescribe pitch to 6 in 2 deg increments
        if UH(jj) > (trans_end - 10)
            offset_x0_hover(10:17) = 60*ones(8,1)*(trans_end-UH(jj))/10; % Prescribe lifting rotor nominal
        end
%offset_x0_hover(10:17)= 65;
        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [8*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
case 6.1 % This is the schedule that works.  Case 6.1 is experimental
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];

        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180;  % Prescribe pitch to 6 in 2 deg increments
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; 60; 95; % Prescribe lifting rotor nominal
         offset_x0_hover(18) =  100; 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';                 
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        % **************************** Transition *************************
        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
%X0_trans = [7*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];

        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);
% FreeVar_trans       =  boolean([ 1   0   0   0   0  0   1   1   1   [1 0 0 0]   [1 0 0 0]   1  ]);

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        offset_x0_trans(1)  = 6*pi/180+UH(jj)/trans_end *3*pi/180;  % Prescribe pitch to 6 in 2 deg increments
        if UH(jj) > (trans_end - 10)
            offset_x0_hover(10:17) = 60*ones(8,1)*(trans_end-UH(jj))/10; % Prescribe lifting rotor nominal
        end
%offset_x0_hover(10:17)= 65;
        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [8*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
    case 6.2 % This is the schedule that works.  Case 6.1 is experimental
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];

        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180;  % Prescribe pitch to 6 in 2 deg increments
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; 60; 95; % Prescribe lifting rotor nominal
         offset_x0_hover(18) =  100; 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';                 
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        % **************************** Transition *************************
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
        if UH(jj) == 55*kts2ft || UH(jj) == 85*kts2ft || UH(jj) == 90*kts2ft
            %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
            X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
            X0_trans(FreeVar) = x; % Set the initial conditions to those found in last iteration
            % Fix ganging...
            X0_trans(11) = X0_trans(10);  X0_trans(13) = X0_trans(12); X0_trans(15) = X0_trans(14);X0_trans(17) = X0_trans(16);
        end



        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        offset_x0_trans(1)  = 6*pi/180+UH(jj)/trans_end *3*pi/180;  % Prescribe pitch to 6 in 2 deg increments
        if UH(jj) > (trans_end - 10)
            offset_x0_hover(10:17) = 60*ones(8,1)*(trans_end-UH(jj))/10; % Prescribe lifting rotor nominal
        end
%offset_x0_hover(10:17)= 65;
        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [8*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
       
    case 6.3 % This is the schedule that works.  Case 6.1 is experimental
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];
        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180;  % Prescribe pitch to 6 in 2 deg increments
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; 60; 95; % Prescribe lifting rotor nominal
         offset_x0_hover(18) =  100; 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';                 
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        
        % **************************** Transition *************************
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  0   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
        
        %if UH(jj) == 55*kts2ft  || UH(jj) == 60*kts2ft || UH(jj) == 65*kts2ft  || UH(jj) == 70*kts2ft || UH(jj) == 80*kts2ft || UH(jj) == 85*kts2ft 
        if UH(jj) == 55*kts2ft  || UH(jj) == 60*kts2ft || UH(jj) == 65*kts2ft  || UH(jj) == 70*kts2ft || UH(jj) == 80*kts2ft || UH(jj) == 85*kts2ft 
            %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
            X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
            X0_trans(FreeVar) = x; % Set the initial conditions to those found in last iteration
            % Fix ganging...
            X0_trans(11) = X0_trans(10);  X0_trans(13) = X0_trans(12); X0_trans(15) = X0_trans(14);X0_trans(17) = X0_trans(16);
        end

        %                      th     phi    p      q      r      flp      ail    elv    rud    rl rt pp 
        %scale_trans         = [2*180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 2*180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';

        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        offset_x0_trans(1)  = 6*pi/180+(UH(jj)-trans_start)/(trans_end-trans_start)*3*pi/180; % Prescribe pitch schedule
        %fprintf(1,'Desired Angle: %6.3g\n', offset_x0_trans(1)*180/pi)

        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [8*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
    case 6.4 % This is the schedule that works.  Case 6.1 is experimental
        % **************************** Hover ******************************
        % Set hover initial conditions 
        %           th           phi       p  q  r  flp         ail       elv       rud       rl              rt              pp 
        X0_hover =  [-2*pi/180;  0*pi/180; 0; 0; 0; -25*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(90,4,1); repmat(90,4,1); 100];
        %                              th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_hover      =  boolean([ 1   1   0   0   0  1   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Prescribe the offset vector for the cost computation
         offset_x0_hover     = zeros(18,1); % Initialize offset to x0 then modify it... 
         offset_x0_hover(1)  = UH(jj)/trans_start *6*pi/180;  % Prescribe pitch to 6 in 2 deg increments
         offset_x0_hover(6)  = (1-UH(jj)/trans_start)*-25*pi/180; % Prescribe flap schedule
         offset_x0_hover(10:17) = 90; 60; 95; % Prescribe lifting rotor nominal
         offset_x0_hover(18) =  100; 88*UH(jj)/trans_start; 100; 30 + 70*UH(jj)/trans_start; 100;  % Prescribe pusher rotor nominal
        
        % Normalized scaling...
        %                      th     phi    p     q      r      flp      ail    elv    rud    rl rt pp 
        scale_hover         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';                 
        gang_hover          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_hover = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
        
        % **************************** Transition *************************
        %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_trans       =  boolean([ 1   0   0   0   0  0   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);

        % Set transition initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
        
        %if UH(jj) == 55*kts2ft  || UH(jj) == 60*kts2ft || UH(jj) == 65*kts2ft  || UH(jj) == 70*kts2ft || UH(jj) == 80*kts2ft || UH(jj) == 85*kts2ft 
%         if UH(jj) == 55*kts2ft  || UH(jj) == 60*kts2ft || UH(jj) == 65*kts2ft  || UH(jj) == 70*kts2ft || UH(jj) >= 75*kts2ft %&& UH(jj) < 90*kts2ft 
%             %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
%             X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
%             X0_trans(FreeVar) = x; % Set the initial conditions to those found in last iteration
%             %X0_trans(FreeVar([7 9])) = [0;0]; % Set rudder and ail to zero
%             % Fix ganging...
%             X0_trans(11) = X0_trans(10);  X0_trans(13) = X0_trans(12); X0_trans(15) = X0_trans(14);X0_trans(17) = X0_trans(16);
%         end

%                      th         phi    p      q      r      flp     ail       elv       rud        rl rt    pp 
%scale_trans         = [2*180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 2*180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
%scale_trans         = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
scale_trans         = [150*180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 100*180/pi 20*180/pi 100*180/pi repmat(1,1,9)]'./(UB-LB)';
if UH(jj) > 55*kts2ft
            X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
            X0_trans(FreeVar) = x; % Set the initial conditions to those found in last iteration
            %X0_trans(FreeVar([7 9])) = [0;0]; % Set rudder and ail to zero
            % Fix ganging...
            X0_trans(11) = X0_trans(10);  X0_trans(13) = X0_trans(12); X0_trans(15) = X0_trans(14);X0_trans(17) = X0_trans(16);
end
        % Prescribe the offset vector for the cost computation.
        offset_x0_trans     = zeros(18,1);
        offset_x0_trans(1)  = 6*pi/180+(UH(jj)-trans_start)/(trans_end-trans_start)*3*pi/180; % Prescribe pitch schedule
        if UH(jj) >= 65*kts2ft && UH(jj) <= 90*kts2ft
            offset_x0_trans(1)  = 7.18*pi/180+(UH(jj)-70*kts2ft)/(89*kts2ft-70*kts2ft)*1.24*pi/180;
            %                      th         phi    p      q      r      flp    ail       elv       rud       rl rt pp 
            scale_trans         = [150*180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 100*180/pi 20*180/pi 100*180/pi repmat(1,1,9)]'./(UB-LB)';
            %scale_trans         = [100*180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
            X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
            X0_trans(FreeVar) = x; % Set the initial conditions to those found in last iteration
            %X0_trans(FreeVar([7 9])) = [0;0]; % Set rudder and ail to zero
            % Fix ganging...
            X0_trans(11) = X0_trans(10);  X0_trans(13) = X0_trans(12); X0_trans(15) = X0_trans(14);X0_trans(17) = X0_trans(16);
%             if UH(jj) >=70*kts2ft && UH(jj) <= trans_end % Try to enforce continuity
% %                 %                                th  phi p   q   r  flp ail elv rud rl          rt         pp 
% %                 FreeVar_trans       =  boolean([ 1   0   0   0   0  0   1   1   1   [1 0 1 0]   [1 0 1 0]   1  ]);
%                 offset_x0_trans([7 8 9]) = x(2:4);
%                 offset_x0_trans([7 9]) = [0;0];
%                 offset_x0_trans([10 12 14 16 18]) = x(5:9);
%                 offset_x0_trans(11) = offset_x0_trans(10);  offset_x0_trans(13) = offset_x0_trans(12); 
%                 offset_x0_trans(15) = offset_x0_trans(14);offset_x0_trans(17) = offset_x0_trans(16);
%             end
%             if UH(jj) >= 80*kts2ft % Load the stored output as ICs (faster only...)
%              a = load('Temp.mat');
%              X0_trans = [6*pi/180;  0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(35,4,1); repmat(35,4,1); 105];
%              X0_trans(FreeVar) = a.t(:,2); % Set the initial conditions to those found in last iteration
%             end
%             if UH(jj) >= 88*kts2ft && UH(jj)<=90*kts2ft % Try to enforce continuity
%                 offset_x0_trans([6 7 8 9]) = x(1:4);
%                 offset_x0_trans([10 12 14 16 18]) = x(5:9);
%                 offset_x0_trans(11) = offset_x0_trans(10);  offset_x0_trans(13) = offset_x0_trans(12); 
%                 offset_x0_trans(15) = offset_x0_trans(14);offset_x0_trans(17) = offset_x0_trans(16);
%             end
        end
        %fprintf(1,'Desired Angle: %6.3g\n', offset_x0_trans(1)*180/pi)

        %                      th     phi    p      q      r      flp    ail    elv    rud    rl rt pp 
        gang_trans          = [zeros(9,1); 1*ones(2,1); 2*ones(2,1); 3*ones(2,1); 4*ones(2,1);0];
        %gang_trans          = [zeros(9,1); 1*ones(4,1); 2*ones(4,1);0];

        % Set the costs (quadratic) for the mycost function
        Q_trans = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);

        % ****************************** Cruise ***************************
        % Set cruise initial conditions
        %           th         phi       p  q  r  flp       ail       elv       rud       rl              rt              pp 
        X0_cruise = [8*pi/180; 0*pi/180; 0; 0; 0; 0*pi/180; 0*pi/180; 0*pi/180; 0*pi/180; repmat(0,4,1); repmat(0,4,1); 150];
        %                               th  phi p   q   r  flp ail elv rud rl          rt         pp 
        FreeVar_cruise      = boolean([ 1   1   0   0   0  0   1   1   1   [0 0 0 0]   [0 0 0 0]   1  ]);
        offset_x0_cruise    = zeros(18,1);
        % Normalized scaling...
        scale_cruise        = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 180/pi repmat(1,1,9)]'./(UB-LB)';
        gang_cruise            = zeros(18,1);
        % Set the costs (quadratic) for the mycost function
        Q_cruise = diag([q_theta q_phi q_p q_q q_r q_del_f q_del_a q_del_e q_del_r q_om_l q_om_t q_om_p]);
    otherwise
        disp('The trim design number specified does not correspond to a design specified!  Error....')
        error;
end