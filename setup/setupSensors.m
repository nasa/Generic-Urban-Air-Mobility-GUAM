%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define sensor characteristics for GL-10 sim
% Quantities based on VectorNav VN-200 and  LightWare SF10
% switch to turn sensor noise on/off is in setupGL.m
% 
% ==> edited setupGL.m to call this function <K. Ackerman 31 Mar 2017> 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Out = setupSensors(SimIn)

% Gyro
Sensor.Gyro.X.bias = 0 * SimIn.Units.deg; 
Sensor.Gyro.X.var  = 0.0035 * SimIn.Units.deg; 
Sensor.Gyro.X.seed = round(586123*rand(1));

Sensor.Gyro.Y.bias = 0 * SimIn.Units.deg; 
Sensor.Gyro.Y.var  = 0.0035 * SimIn.Units.deg; 
Sensor.Gyro.Y.seed = round(586123*rand(1));

Sensor.Gyro.Z.bias = 0 * SimIn.Units.deg; 
Sensor.Gyro.Z.var  = 0.0035 * SimIn.Units.deg; 
Sensor.Gyro.Z.seed = round(586123*rand(1));

Sensor.Gyro.Res = 0.02 * SimIn.Units.deg; 
Sensor.Gyro.Lim = 2000 * SimIn.Units.deg; 
Sensor.Gyro.Ts = 0.005 * SimIn.Units.s; 
Sensor.Gyro.Tn = 0.005 * SimIn.Units.s; 

% Accelerometer
Sensor.Accel.X.bias = 0 * SimIn.Units.g0; 
Sensor.Accel.X.var  = 0.00014 * SimIn.Units.g0; 
Sensor.Accel.X.seed = round(586123*rand(1));

Sensor.Accel.Y.bias = 0 * SimIn.Units.g0; 
Sensor.Accel.Y.var  = 0.00014 * SimIn.Units.g0; 
Sensor.Accel.Y.seed = round(586123*rand(1));

Sensor.Accel.Z.bias = 0 * SimIn.Units.g0; 
Sensor.Accel.Z.var  = 0.00014 * SimIn.Units.g0; 
Sensor.Accel.Z.seed = round(586123*rand(1));

Sensor.Accel.Res = 0.0005 * SimIn.Units.g0; 
Sensor.Accel.Lim = 16 * SimIn.Units.g0; 
Sensor.Accel.Ts = 0.005 * SimIn.Units.s; 
Sensor.Accel.Tn = 0.005 * SimIn.Units.s; 

% Quaternion
Sensor.Quat.bias = 0 * SimIn.Units.deg;  
Sensor.Quat.var  = 0.5 * SimIn.Units.deg; 
Sensor.Quat.seed = round(586123*rand(1));
Sensor.Quat.dir  = [0.5853 0.5497 0.9172]'/norm([0.5853 0.5497 0.9172]');

Sensor.Quat.Ts = 0.005 * SimIn.Units.s;
Sensor.Quat.Tn = 1 * SimIn.Units.s;

% GPS
Re = 2.0904e7 * SimIn.Units.ft;
Sensor.GPS.r.bias = 0 * SimIn.Units.m;
Sensor.GPS.r.var  = 2.5 * SimIn.Units.m;
Sensor.GPS.r.seed = round(586123*rand(1));

Sensor.GPS.dir.min = 0 * SimIn.Units.deg;
Sensor.GPS.dir.max = 360 * SimIn.Units.deg;
Sensor.GPS.dir.seed = round(586123*rand(1));

Sensor.GPS.Lat.Res = (1/Re) * (0.001*SimIn.Units.m) * SimIn.Units.ft;
Sensor.GPS.Lon.Res = (1/(Re*cosd(37.102935))) * (0.001*SimIn.Units.m) * SimIn.Units.ft;

Sensor.GPS.alt.bias = 0 * SimIn.Units.m;
Sensor.GPS.alt.var  = 5 * SimIn.Units.m;
Sensor.GPS.alt.seed = round(586123*rand(1));
Sensor.GPS.alt.Lim  = 50e3 * SimIn.Units.ft;
Sensor.GPS.alt.Res = 0.001 * SimIn.Units.m;

Sensor.GPS.Vn.bias = 0 * SimIn.Units.m;
Sensor.GPS.Vn.var  = 0.05 * SimIn.Units.m;
Sensor.GPS.Vn.seed = round(586123*rand(1));

Sensor.GPS.Ve.bias = 0 * SimIn.Units.m;
Sensor.GPS.Ve.var  = 0.05 * SimIn.Units.m;
Sensor.GPS.Ve.seed = round(586123*rand(1));

Sensor.GPS.Vd.bias = 0 * SimIn.Units.m;
Sensor.GPS.Vd.var  = 0.05 * SimIn.Units.m;
Sensor.GPS.Vd.seed = round(586123*rand(1));

Sensor.GPS.vel.Res = 0.001 * SimIn.Units.m;
Sensor.GPS.vel.Lim = 500 * SimIn.Units.m;

Sensor.GPS.Tn = 0.2*10 * SimIn.Units.s;
Sensor.GPS.vel.Tn = 0.2 * SimIn.Units.s;
Sensor.GPS.Ts = 0.005 * SimIn.Units.s;

% Laser Altimiter
Sensor.LaserAlt.bias = 0 * SimIn.Units.m;
Sensor.LaserAlt.var  = 0.04 * SimIn.Units.m;
Sensor.LaserAlt.seed = round(586123*rand(1));
Sensor.LaserAlt.Res  = 0.01 * SimIn.Units.m;
Sensor.LaserAlt.Lim  = 50 * SimIn.Units.m;

Sensor.LaserAlt.Tn = 0.005 * SimIn.Units.s;
Sensor.LaserAlt.Ts = 0.005 * SimIn.Units.s;

Out = Sensor;

end




