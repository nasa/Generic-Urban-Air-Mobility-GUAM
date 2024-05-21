% Propeller data taken from the UIUC propeller data base

data1 = load('apcsf_10x4p7_4014.csv');
data2 = load('apcsf_10x4p7_4997.csv');
data3 = load('apcsf_10x4p7_5018.csv');
data4 = load('apcsf_10x4p7_6020.csv');
data5 = load('apcsf_10x4p7_6023.csv');
data6 = load('apcsf_10x4p7_6512.csv');
data7 = load('apcsf_10x4p7_6513.csv');


J = [ data1(:,1); data2(:,1); data3(:,1); data4(:,1); data5(:,1); data6(:,1); data7(:,1) ];
Ct = [ data1(:,2); data2(:,2); data3(:,2); data4(:,2); data5(:,2); data6(:,2); data7(:,2) ];
Cp = [ data1(:,3); data2(:,3); data3(:,3); data4(:,3); data5(:,3); data6(:,3); data7(:,3) ];

Ct_p = polyfit(J,Ct,2);
Cp_p = polyfit(J,Cp,2);

Jb = 0:0.01:1;

figure;
plot(J,Ct,'.', Jb, polyval(Ct_p,Jb));
grid on
zoom on

figure
plot(J,Cp,'.',Jb, polyval(Cp_p,Jb));
grid on;
zoom on

APCSF_10x4p7_coef = [Ct_p' Cp_p'];

save('APCSF_10x4p7_coef.mat', 'APCSF_10x4p7_coef');

