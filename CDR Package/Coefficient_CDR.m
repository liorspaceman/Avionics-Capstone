clc

%% Analytical Transfer Functions
la=0.65405; %converted 25.75 inches to m 
lb=0.460375; %converted 18.125 inches to m
lh=6.985*0.0254;
mh=1.442; % Mass of Heli 1
wh=1.442*9.81; % Weight of Heli 1
wc=1.914*9.81; % Weight of Counter weight
Mc=1.914;
Mf=1.442*0.5;
Mb=1.442*0.5;


Rm = 0.83;
Je = 1.07; %top 
Jt = 1.07; %side
Jp = 0.04538;
lh = 0.177419; % converted 6.985 inches into m
Kmt = 0.019;% Motor current torque constant
Jpm = 6.4516e-5; %Propeller moment of inertia
Kf = 0.1344; % Force Lift Constant [N/V]
Kt = 0.0206; % Motor Torque Constant [Nm/A]
Fw = 0.0034; % Force speed Constant [N/rad(s)] 
Kpt = 0.0002; % Torque speed Constant [Nm/rad(s)] 
Kb = 0.0228;
Keff = 0.15*9.81;
K_CABLE=3; 

F = Kt*Fw;
V = [Rm*Jpm,(Kpt*Rm)+(Kt*Kb)];
MotorCof = tf(F,V)

E = la;
Fsum = [Je,0,0];
G_Elev = tf(E,Fsum) 

Lamb = -Keff*la;
Rhotrav = [Jt,0,0];
G_Trav = tf(Lamb,Rhotrav) %PID 
G_Trav_pos = tf(Lamb*-1,Rhotrav)

Rhopit = lh; 
Fdiff = [Jp,0,0];
G_Pitch = tf(Rhopit,Fdiff)

Elev_sys = MotorCof*G_Elev %PID
Pitch_sys = MotorCof*G_Pitch %PD

%% Elevation Design
load G1_elevData1

time1=elevationData1.time(104:2479);
volts1=elevationData1.signals.values(104:2479,1);
elev1=elevationData1.signals.values(104:2479,2);

data_1=iddata(elev1,volts1,0.01);

G_Elev2=tfest(data_1,2,0)
G_Elev3=tfest(data_1,3,0) 
G_Elev4=tfest(data_1,4,0)

%% State space conversion for Elevation 
num= 0.03382; %Experimental Elevation State-space
den= [1,0.1057,1.361];
[A,B,C,D]=tf2ss(num,den);
sys=ss(A,B,C,D);

%% Travel and Pitch Design

load Time
load PitchData
load TravelData
load Travel_Pitch
load VdiffData_Pitch
load VdiffData_Travel
load MotorLift
load MotorVoltage

MotorLift_Data=table2array(MotorLift);
MotorVoltage_Data=table2array(MotorVoltage);
Time=table2array(Time);
Trav_Data=table2array(TravelData);
Pitch_Data=table2array(PitchData);
Travel_Pitch_Data=table2array(Travel_Pitch);
Vdiff_Data_Pitch=table2array(VdiffData_Pitch);
Vdiff_Data_Travel=table2array(VdiffData_Travel);

%data_4=iddata(Pitch_Data(49982:2:172506),Vdiff_Data_Pitch(49982:2:172506),0.01);
%data_5=iddata(Trav_Data(2:2:125002),Travel_Pitch_Data(2:2:125002),0.01); 
data_6=iddata(MotorLift_Data(2:end),MotorVoltage_Data(2:end),1);

Motor_Exp=tfest(data_6,1,0);
Elev_Final=minreal(G_Elev3/Motor_Exp);
%G_Pitch_new_2pole_1=tfest(data_4,2,0);
%G_Travel_new_2pole_1=tfest(data_5,2,0);


%% Finding Ki, Kd, Kp

load G_Pitch_new_2pole_1 %with new data range
load G_Travel_new_2pole_1 %with new data range

load C_Elev2 
load C_Pitch_new_2pole_1 % with new data range
load C_Travel_new_2pole_2 % with new data range

fprintf('\nElevation2:')
PID_Elev_2 = pid(C_Elev2)
Kd_2 = PID_Elev_2.kd
Kp_2 = PID_Elev_2.kp
Ki_2 = PID_Elev_2.ki

fprintf('\nTravel_2pole:')
PID_Travel_2pole = pid(C_Travel_new_2pole_2)
Kdt_2pole = PID_Travel_2pole.kd
Kpt_2pole = PID_Travel_2pole.kp
Kit_2pole = PID_Travel_2pole.ki

fprintf('\nPitch2pole:')
PID_Pitch_new_2pole = pid(C_Pitch_new_2pole_1)
Kdp_2p = PID_Pitch_new_2pole.kd
Kpp_2p = PID_Pitch_new_2pole.kp
Kip_2p = PID_Pitch_new_2pole.ki


%% Closed loop transfer function plots 
Cltf_Elev2 = feedback(C_Elev2*G_Elev2,1)

Cltf_Pitch_new_2pole_1 = feedback(C_Pitch_new_2pole_1*G_Pitch_new_2pole_1,1)

Cltf_Travel_new_2pole_1 = feedback(C_Travel_new_2pole_2*G_Travel_new_2pole_1,1)

%% Defining parameters

fprintf('\nElevation2:')
SysElev2=stepinfo(Cltf_Elev2);
Tr_E_2 = SysElev2.RiseTime 
Ts_E_2 = SysElev2.SettlingTime 
Os_E_2 = SysElev2.Overshoot


fprintf('\nTravel_new_2pole:')
SysTrav_2pole_1=stepinfo(Cltf_Travel_new_2pole_1);
Tr_Travel_new_2pole_1= SysTrav_2pole_1.RiseTime 
Ts_Travel_new_2pole_1 = SysTrav_2pole_1.SettlingTime 
Os_Travel_new_2pole_1 = SysTrav_2pole_1.Overshoot


fprintf('\nPitch2pole:')
SysPitch_Pitch_new_2pole_1=stepinfo(Cltf_Pitch_new_2pole_1);
Tr_Pitch_new_2pole_1 = SysPitch_Pitch_new_2pole_1.RiseTime 
Ts_Pitch_new_2pole_1 = SysPitch_Pitch_new_2pole_1.SettlingTime 
Os_Pitch_new_2pole_1 = SysPitch_Pitch_new_2pole_1.Overshoot






