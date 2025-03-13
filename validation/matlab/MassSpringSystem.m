close
clear
clc

m = 1.0;
k = 1.0;
c = 0.5;
kp = 1.0;
ki = 1.0;
kd = 1.0;

load("../data/matlab/SIMULINK_Output.mat")

t_log       = SIMULINK_Output.Time;
x1_log      = SIMULINK_Output.Data(:,1);
x2_log      = SIMULINK_Output.Data(:,2);
ctrl_log    = SIMULINK_Output.Data(:,3);
ref_log     = SIMULINK_Output.Data(:,4);

results_matlab = [t_log,x1_log,x2_log,ctrl_log,ref_log];

writematrix(results_matlab, '../data/matlab/MassSpringDamper.csv');