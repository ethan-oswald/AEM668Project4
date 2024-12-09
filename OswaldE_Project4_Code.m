clear;
clc;
close all;


%% Part B
%Define actuator
actuator.omega = 150;
actuator.DR = 0.7;
actuator.tf = tf(actuator.omega^2, [1 2*actuator.DR*actuator.omega * actuator.omega^2]);
actuator.tf.OutputName = "Deflection";
actuator.A = [0,1;-actuator.omega^2,-2*actuator.omega*actuator.DR];
actuator.B = [0;actuator.omega^2];
actuator.C = [1,0;0,1];
actuator.D = [0;0];
actuator.ss = ss(actuator.A,actuator.B,actuator.C,actuator.D);
actuator.ss.InputName = "Commanded Deflection";
actuator.ss.OutputName = ["Deflection", "Deflection Rate"];

%Define trim values
trim.M = 3;
trim.alpha = 0* pi/180;

%Use function to find plant at trim values
[plant.A,plant.B,plant.C,plant.D] = fun(trim.alpha, trim.M);
plant.ss = ss(plant.A,plant.B,plant.C,plant.D);

%Name inputs and outputs of blocks
plant.ss.InputName = "Deflection";
plant.ss.OutputName = ["Angle of Attack", "Pitch", "Acceleration"];

%Connect everything besides the controller into a generalized plant
plant.generalized = connect(actuator.ss,plant.ss, "Commanded Deflection",["Acceleration", "Angle of Attack", "Pitch","Deflection", "Deflection Rate"]);

%Set poles
p = [-100, -200, -50,-5];

%Create gain matrix and controller based on poles
K = place(plant.generalized.A, plant.generalized.B, p);
controller.K = tf(-K);
controller.K.InputName = ["Deflection", "Deflection Rate", "Angle of Attack", "Pitch"];
controller.K.OutputName = "CD_Feedback";

%Create sumblock for use by the controller
system.S = sumblk("Commanded Deflection = CD + CD_Feedback");

%Create the plant controlled based on AOA, Pitch,Deflection, and Deflection
%Rate
plant.controlled = connect(system.S,controller.K, plant.generalized, "CD", "Acceleration");

%Define Tracking Error
system.tracking_error = tf(1, [1 0]);
system.tracking_error.InputName = "e";
system.tracking_error.OutputName = "CD_uncontrolled";

%Define a fifth gain for the acceleration integral
K5 = 0.5;
controller.K5 = tf(K5);
controller.K5.InputName = "CD_uncontrolled";
controller.K5.OutputName = "CD";

%Create a sumblock for the error
system.S2 = sumblk("e = Step - Acceleration");
%Create the final system
system.total = connect(system.S2, system.tracking_error, controller.K5, plant.controlled, "Step", "Acceleration");

%Combine all five gains into one single controller to be called later
controller.total = connect(controller.K5, controller.K, system.S, ["CD_uncontrolled", "Deflection", "Deflection Rate", "Angle of Attack", "Pitch"], "Commanded Deflection");

%Determine and output the controllability of the generalized plant
plant.cntrl = rank(ctrb(plant.generalized));
fprintf("The Rank of the Controllability of the Generalized Plant is %5.2f.\n" + ...
    "Since this value matches the number of states in the system, the generalized plant is controllable.\n\n", plant.cntrl);

%Define the open loop and find/output gain/phase matrix
open_loop = connect(controller.total, plant.generalized, "CD_uncontrolled", "Acceleration");
[gm, pm] = margin(open_loop);
fprintf("The gain and phase margins of the open loop are %5.2f dB and %5.2f degrees.\n\n", gm, pm);

%% Part C

%Run Simulation with increasing step size and plot response
for i = 1:5
    step_size = 0.015 * i;
    Simulation = sim("OswaldE_Project4_Sim");
    az.linear_series = Simulation.az_linear;
    az.nonlinear_series = Simulation.az_nonlinear;
    time = az.linear_series.time;
    az.linear = az.linear_series.data;
    az.nonlinear = az.nonlinear_series.data;
    figure(i)
    plot(time,az.linear);
    hold on
    plot(time, az.nonlinear)
    title(sprintf('Acceleration Step Response for a step of %1.3f g', step_size));
    legend("Linear", "Nonlinear", 'Location', 'southeast');
end

fprintf("The nonlinear step responses for steps of 0.015, 0.03, 0.045, 0.06, and 0.075 g are shown.\n" + ...
    "For the smaller step sizes, the linear and nonlinear systems agree, but the nonlinear system\n" + ...
    "takes slightly longer to settle. As the step value increases, the nonlinear system continues to \n" + ...
    "respond proportionally slower to the linear system.\n\n");


%% Part D
%Create uncertain range of AOA and uncertain plant
uncertain.alpha = ureal('alpha',5*pi/180,'Range', [0,10*pi/180], 'Autosimplify','full');
uncertain.plant = fun2(uncertain.alpha,trim.M);
simplify(uncertain.plant, 'full');
uncertain.plant.InputName = "Deflection";
uncertain.plant.OutputName = ["Angle of Attack", "Pitch", "Acceleration"];

%Create weight for the actuator
actuator.weight = tf(0.6);
actuator.weight.InputName = "Commanded Deflection";
actuator.weight.OutputName = "Actuator Weight Output";
actuator.tf.InputName = "Uncertain Commanded Deflection";
actuator.ss.InputName = "Uncertain Commanded Deflection";

%Create weight for the vibration
vibration.weight = tf([150 13000 70000 48000], [1 2000 2000000 62000000]);
vibration.weight.InputName = "Deflection";
vibration.weight.OutputName = "Vibration Weight Output";

%Create the uncertain actuator
uncertain.actuator = ultidyn('actuator', 1);
uncertain.actuator.InputName = "Actuator Weight Output";
uncertain.actuator.OutputName = "Actuator Uncertainty Output";

%Create the uncertain vibration
uncertain.vibration = ultidyn('vibration', 1);
uncertain.vibration.InputName = "Vibration Weight Output";
uncertain.vibration.OutputName = "Vibration Uncertainty Output";

%Create the sum blocks for the uncertain elements
uncertain.S1 = sumblk("Uncertain Commanded Deflection = Commanded Deflection + Actuator Uncertainty Output");
uncertain.S2 = sumblk("Uncertain Pitch = Pitch + Vibration Uncertainty Output");

%Create the full uncertain system and output bode plots
uncertain.system = connect(actuator.weight,uncertain.actuator,uncertain.S1,actuator.tf, vibration.weight, uncertain.vibration,uncertain.plant,uncertain.S2, "Commanded Deflection", ["Angle of Attack", "Uncertain Pitch", "Acceleration"]);
figure(6)
bode(uncertain.system)
title("Bode Plot for the Uncertain System")

%% Part E
%Alter Controller inputs for uncertain system
controller.total.InputName = ["CD_uncontrolled", "Deflection", "Deflection Rate", "Angle of Attack", "Uncertain Pitch"];

%Create Controlled Uncertain System
uncertain.system_controlled = connect(system.S2, system.tracking_error, controller.total, actuator.weight,uncertain.actuator,uncertain.S1,actuator.ss, vibration.weight, uncertain.vibration,uncertain.plant,uncertain.S2, "Step", "Acceleration");

%Find, output, and comment on stability margin and worst case gain for full system
[stabmarg, wcu1] = robstab(uncertain.system_controlled);
fprintf("The stability margin of the controlled uncertain system has a lower bound at %5.4f \n" + ...
    "and an upper bound at %5.4f. These values being above one means it is stable for our range of uncertainties.\n\n", stabmarg.LowerBound, stabmarg.UpperBound);
[wcg, wcu2] = wcgain(uncertain.system_controlled);
fprintf("The worst case gain for the controlled uncertain system has a lower bound at %5.2f dB\n" + ...
    "and an upper bound at %5.2f dB. This value being so close to 1 means that even in the worst case,\n" + ...
    "the output will only be slightly amplified relative to the input, and does not pose a risk of instability.\n", wcg.LowerBound, wcg.UpperBound);

%% Functions
function [A, B, C, D] = fun(AOA, M)
%Declare parameters
vehicle.S_ref = .44;
vehicle.diameter = .75;
vehicle.mass = 13.98;
vehicle.inertia_yy = 182.5;
vehicle.g = 32.2;
vehicle.gamma = 1.4;
vehicle.pressure = 973.3;
vehicle.n0 = -.034 * 180/pi;
vehicle.n1_0 = -0.3392* 180/pi;
vehicle.n1_M = 0.0565333* 180/pi;
vehicle.n2 = -0.0094457* (180/pi)^2;
vehicle.n3 = .000103* (180/pi)^3;
vehicle.m0 = -.206* 180/pi;
vehicle.m1_0 = -.357* 180/pi;
vehicle.m1_M = .136* 180/pi;
vehicle.m2 = -.019546* (180/pi)^2;
vehicle.m3 = .000215* (180/pi)^3;

%Calculate Q and declare speed of sound
Q = 1/2 * vehicle.gamma * vehicle.pressure * M^2;
C_s = 1036.4;

%find delta value by setting Cm equal to 0
delta = -(AOA*(vehicle.m3*abs(AOA)^2 + vehicle.m2*abs(AOA) + (vehicle.m1_M*M + vehicle.m1_0)))/vehicle.m0;

%Calculate Cn(Cm is not needed for linear system)
Cn = AOA*(vehicle.n3*abs(AOA)^2 + vehicle.n2*abs(AOA) + (vehicle.n1_M*M + vehicle.n1_0)) + vehicle.n0*delta;

%Find derivatives of Cn and Cm with respect to all needed variables
dCn_dalpha = 2*vehicle.n2*abs(AOA)+3*vehicle.n3*AOA^2 + vehicle.n1_0 + M*vehicle.n1_M;
dCm_dalpha = 2*vehicle.m2*abs(AOA)+3*vehicle.m3*AOA^2 + vehicle.m1_0 + M*vehicle.m1_M;

dCn_dq = 0;
dCn_ddelta = vehicle.n0;
dCm_ddelta = vehicle.m0;

%Find derivatives of f1 and f2 with respect to all needed variables
df1_dalpha = Q*vehicle.S_ref/(vehicle.mass*C_s*M) * (-Cn*sin(AOA) + dCn_dalpha*cos(AOA));
df1_dq = 1;
df1_ddelta = cos(AOA)*Q*vehicle.S_ref/(vehicle.mass*C_s*M) * dCn_ddelta;

df2_dalpha = Q*vehicle.S_ref*vehicle.diameter/vehicle.inertia_yy * (dCm_dalpha);
df2_dq = 0;
df2_ddelta = Q*vehicle.S_ref*vehicle.diameter/vehicle.inertia_yy * (dCm_ddelta);

%Create A,B,C,D matrices
A = [df1_dalpha, df1_dq; df2_dalpha, df2_dq];
B = [df1_ddelta; df2_ddelta];
C = [1, 0; 0, 1; (pi/180 * (Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_dalpha), ((Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_dq)];
D = [0;0; (pi/180 * (Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_ddelta)];
end

function [state_space] = fun2(AOA,M)
%This function is fundamentally similar to function 1 but with sine and
%cosine approximations, and abs() removed, as the uncertain range is all
%greater than 0

vehicle.S_ref = .44;
vehicle.diameter = .75;
vehicle.mass = 13.98;
vehicle.inertia_yy = 182.5;
vehicle.g = 32.2;
vehicle.gamma = 1.4;
vehicle.pressure = 973.3;
vehicle.n0 = -.034 * 180/pi;
vehicle.n1_0 = -0.3392* 180/pi;
vehicle.n1_M = 0.0565333* 180/pi;
vehicle.n2 = -0.0094457* (180/pi)^2;
vehicle.n3 = .000103* (180/pi)^3;
vehicle.m0 = -.206* 180/pi;
vehicle.m1_0 = -.357* 180/pi;
vehicle.m1_M = .136* 180/pi;
vehicle.m2 = -.019546* (180/pi)^2;
vehicle.m3 = .000215* (180/pi)^3;

Q = 1/2 * vehicle.gamma * vehicle.pressure * M^2;
C_s = 1036.4;

C = 1 - AOA^2/2;
S = AOA - AOA^3/6;

delta = -(AOA*(vehicle.m3*AOA^2 + vehicle.m2*AOA + (vehicle.m1_M*M + vehicle.m1_0)))/vehicle.m0;
Cn = AOA*(vehicle.n3*AOA^2 + vehicle.n2*AOA + (vehicle.n1_M*M + vehicle.n1_0)) + vehicle.n0*delta;

dCn_dalpha = 2*vehicle.n2*AOA+3*vehicle.n3*AOA^2 + vehicle.n1_0 + M*vehicle.n1_M;
dCm_dalpha = 2*vehicle.m2*AOA+3*vehicle.m3*AOA^2 + vehicle.m1_0 + M*vehicle.m1_M;

dCn_dq = 0;
dCn_ddelta = vehicle.n0;
dCm_ddelta = vehicle.m0;

df1_dalpha = Q*vehicle.S_ref/(vehicle.mass*C_s*M) * (-Cn*S + dCn_dalpha*C);
df1_dq = 1;
df1_ddelta = C*Q*vehicle.S_ref/(vehicle.mass*C_s*M) * dCn_ddelta;

df2_dalpha = Q*vehicle.S_ref*vehicle.diameter/vehicle.inertia_yy * (dCm_dalpha);
df2_dq = 0;
df2_ddelta = Q*vehicle.S_ref*vehicle.diameter/vehicle.inertia_yy * (dCm_ddelta);

A = [df1_dalpha, df1_dq; df2_dalpha, df2_dq];
B = [df1_ddelta; df2_ddelta];
C = [1, 0; 0, 1; (pi/180 * (Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_dalpha), ((Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_dq)];
D = [0;0; (pi/180 * (Q*vehicle.S_ref/(vehicle.mass*vehicle.g))*dCn_ddelta)];

state_space = ss(A,B,C,D);
end