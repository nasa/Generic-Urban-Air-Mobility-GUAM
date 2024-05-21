function Out = setupAdaptive(SimIn)

Out.AdaptOn = 0;
Out.PWC = false;  

if Out.PWC
    Out.Ts = 1/200;
    
    Out.Aelon = -3*2*pi*eye(4);
    Out.Aelat = -3*2*pi*eye(4);
    
    Out.filtAlon = -3*2*pi*eye(3);
    Out.filtBlon = 3*2*pi*eye(3);
    Out.filtClon = -eye(3);
    Out.filtDlon = zeros(3);
    
    Out.filtAlat = -3*2*pi*eye(3);
    Out.filtBlat = 3*2*pi*eye(3);
    Out.filtClat = -eye(3);
    Out.filtDlat = zeros(3);
    
    Iinvlon = Out.Aelon/(expm(Out.Aelon*Out.Ts)-eye(size(Out.Aelon)));
    Out.Kadlon = -Iinvlon*expm(Out.Aelon*Out.Ts);
    Out.KadIlon = -Iinvlon;

    Iinvlat = Out.Aelat/(expm(Out.Aelat*Out.Ts)-eye(size(Out.Aelat)));
    Out.Kadlat = -Iinvlat*expm(Out.Aelat*Out.Ts);
    Out.KadIlat = -Iinvlat;
else
    Out.Ts = -1;
    Out.Aelon = zeros(4);
    Out.Aelat = zeros(4);
    
    Out.filtAlon = [];
    Out.filtBlon = [];
    Out.filtClon = [];
    Out.filtDlon = -eye(3);
    
    Out.filtAlat = [];
    Out.filtBlat = [];
    Out.filtClat = [];
    Out.filtDlat = -eye(3);
    
    Out.Kadlon = -3*2*pi*eye(4);
    Out.KadIlon = zeros(4);

    Out.Kadlat = -3*2*pi*eye(4);
    Out.KadIlat = zeros(4);
end