function [XEQ] = show_trim(trim_file,ONE_PLOT)
  % plot the trim file
  % trim_file - *.mat or array 
  % set ONE_PLOT for single plot
  

  if ischar(trim_file)
      load(trim_file);
  else 
      XEQ = trim_file;
  end

  [MM, ~, NN] = size(XEQ);

  LVL = floor(NN/2)+1;

  ft2kts = 0.592484;

  % % Build a trim table for easy viewing
  % Velocity = XEQ(:,1,LVL)*0.592484;
  % Flight_Path = atan2(-XEQ(:,2,LVL),XEQ(:,1))*180/pi;
  % Pitch_Angle = XEQ(:,4,LVL)*180/pi;
  % Elevator = XEQ(:,10,LVL)*180/pi;
  % Thrust_Pusher = XEQ(:,20,LVL);
  % Thrust_Trailing = mean(XEQ(:,16:19,LVL),2);
  % Thrust_Leading = mean(XEQ(:,12:15,LVL),2);
  % TRIM_TABLE = table(Velocity, Flight_Path, Pitch_Angle, Elevator, Thrust_Pusher, Thrust_Leading, Thrust_Trailing);
  % TRIM_TABLE

  % OM_L = squeeze(XEQ(:,12:15,:));
  % OM_T = squeeze(XEQ(:,16:19,:));
  % OM_P = squeeze(XEQ(:,20,:));
  % TH   = squeeze(XEQ(:, 4,:));
  % PHI  = squeeze(XEQ(:, 5,:));
  % DELE = squeeze(XEQ(:,10,:));

  Velocity = XEQ(:,1,LVL)*0.592484;
  Flight_Path = atan2(-XEQ(:,2,LVL),XEQ(:,1))*180/pi;
  Pitch_Angle = XEQ(:,4,LVL)*180/pi;
  Elevator = XEQ(:,11,LVL)*180/pi;
%   Thrust_Pusher = XEQ(:,20,LVL);
  Thrust_Pusher = XEQ(:,21,LVL);
%   Thrust_Trailing = mean(XEQ(:,16:19,LVL),2);
%   Thrust_Leading = mean(XEQ(:,12:15,LVL),2);
  Thrust_Trailing = mean(XEQ(:,13:16,LVL),2);
  Thrust_Leading = mean(XEQ(:,17:20,LVL),2);
  TRIM_TABLE = table(Velocity, Flight_Path, Pitch_Angle, Elevator, Thrust_Pusher, Thrust_Leading, Thrust_Trailing);
  TRIM_TABLE

  OM_L = squeeze(XEQ(:,13:16,:));
  OM_T = squeeze(XEQ(:,17:20,:));
  OM_P = squeeze(XEQ(:,21,:));
  TH   = squeeze(XEQ(:, 4,:));
  PHI  = squeeze(XEQ(:, 5,:));
  DELF = squeeze(XEQ(:,9,:));
  DELA = squeeze(XEQ(:,10, :));
  DELE = squeeze(XEQ(:,11,:));
  DELR = squeeze(XEQ(:,12,:));


  %% Make some nice plots
  UH = squeeze(XEQ(:,1,:));
  WH = squeeze(XEQ(:,2,:));

  for jj = 1:NN
    for ii = 1:MM
      Rot = Rx(PHI(ii,jj))*Ry(TH(ii,jj));
      vb = Rot*[UH(ii,jj); 0; WH(ii,jj)];
      ALF(ii,jj) = atan2(vb(3),vb(1));
    end
  end

  if nargin < 2
    ONE_PLOT = 0;
  end

  if ONE_PLOT
    figure
    subplot(2,3,4)
    plot(UH*ft2kts, OM_L*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('leading rotors [RPM]', 'fontsize',20)
    legend('LT Outer','LT Inner', 'RT Inner','RT Outer')
    %legend({'$\omega_{lead}$','$\omega_{trail}$','$\omega_{pusher}$'},'fontsize',15)

    subplot(2,3,5)
    plot(UH*ft2kts, OM_T*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('trailing rotors [RPM]', 'fontsize',20)
    legend('LT Outer','LT Inner', 'RT Inner','RT Outer')

    subplot(2,3,6)
    plot(UH*ft2kts, OM_P*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('pusher prop [RPM]', 'fontsize',20)

    subplot(2,3,1)
    plot(UH*ft2kts, PHI*180/pi,'o-')
    grid on 
    zoom on
    ylabel('roll [deg]', 'fontsize',20)
    %legend({'$\theta$','$\delta_e$'},'fontsize',15)

    subplot(2,3,2)
    plot(UH*ft2kts, TH*180/pi,'o-')
    grid on 
    zoom on
    ylabel('pitch [deg]', 'fontsize',20)
    %legend({'$\theta$','$\delta_e$'},'fontsize',15)

    subplot(2,3,3)
    plot(UH*ft2kts, [DELF DELA DELE DELR]*180/pi,'o-')
    grid on 
    zoom on
    ylabel('surfaces [deg]', 'fontsize',20)
    legend({'f','a','e','r'},'fontsize',15)

  else

    figure
    plot(UH*ft2kts, OM_L*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('leading rotors [RPM]', 'fontsize',20)
    legend('LT Outer','LT Inner', 'RT Inner','RT Outer')

    %legend({'$\omega_{lead}$','$\omega_{trail}$','$\omega_{pusher}$'},'fontsize',15)

    figure
    plot(UH*ft2kts, OM_T*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('trailing rotors [RPM]', 'fontsize',20)
    legend('LT Outer','LT Inner', 'RT Inner','RT Outer')


    figure
    plot(UH*ft2kts, OM_P*60/2/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('pusher prop [RPM]', 'fontsize',20)

    figure
    plot(UH*ft2kts, ALF*180/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('angle of attack [deg]', 'fontsize',20)
    %legend({'$\theta$','$\delta_e$'},'fontsize',15)

    figure
    plot(UH*ft2kts, TH*180/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('pitch [deg]', 'fontsize',20)
    %legend({'$\theta$','$\delta_e$'},'fontsize',15)

    figure
    plot(UH*ft2kts, DELE*180/pi,'o-')
    grid on 
    zoom on
    xlabel('ground speed [kts]', 'fontsize',20)
    ylabel('elevator [deg]', 'fontsize',20)
    %legend({'$\theta$','$\delta_e$'},'fontsize',15)
  end

