clc;clear all;close all;
load('04-12-19_04.45 765_LA92_40degC_Turnigy_Graphene.mat')
LiPoly.RecordingTime            = meas.Time;
LiPoly.Measured_Voltage         = meas.Voltage;
LiPoly.Measured_Current         = meas.Current;
LiPoly.Measured_Temperature     = meas.Battery_Temp_degC;
nominalCap                      = 4.81;% Battery capacity in Ah taken from data.
LiPoly.Measured_SOC             = (nominalCap + meas.Ah).*100./nominalCap;  % Calculate the SOC using Coloumb Counting for comparison
% Resample input data
LiPoly.RecordingTime            = LiPoly.RecordingTime(1:10:end);
LiPoly.Measured_Voltage         = LiPoly.Measured_Voltage(1:10:end);
LiPoly.Measured_Current         = LiPoly.Measured_Current(1:10:end);
LiPoly.Measured_Temperature     = LiPoly.Measured_Temperature(1:10:end);
LiPoly.Measured_SOC             = LiPoly.Measured_SOC(1:10:end);

% Current Definition: (+) Discharging, (-) Charging
LiPoly.Measured_Current_R       = - LiPoly.Measured_Current;
% Converting seconds to hours
LiPoly.RecordingTime_Hours      = LiPoly.RecordingTime/3600;

[SOC_Estimated, Vt_Estimated, Vt_Error] = EKF_function(LiPoly.Measured_Current_R, LiPoly.Measured_Voltage, LiPoly.Measured_Temperature);

%plotting the results
% Terminal Voltage Measured vs. Estimated
figure
plot(LiPoly.RecordingTime_Hours,LiPoly.Measured_Voltage);
hold on
plot(LiPoly.RecordingTime_Hours,Vt_Estimated);
hold off;
legend('Measured','Estimated EKF');
ylabel('Terminal Voltage[V]');
xlabel('Time[hr]');
title('Measured vs. Estimated Terminal Voltage (V) at 0 Deg C')
grid minor

% Terminal Voltage Error
figure
plot(LiPoly.RecordingTime_Hours,Vt_Error);
legend('Terminal Voltage Error');
ylabel('Terminal Voltage Error');
xlabel('Time[hr]');

% SOC Coulomb Counting vs. Estimated
figure
plot (LiPoly.RecordingTime_Hours,LiPoly.Measured_SOC);
hold on
plot (LiPoly.RecordingTime_Hours,SOC_Estimated*100);
hold off;
legend('Coulomb Counting','Estimated EKF');
ylabel('SOC[%]');xlabel('Time[hr]');
title('Coulomb Counting vs. SOC Estimated at 0 Deg C')
grid minor

% SOC Error
figure
plot(LiPoly.RecordingTime_Hours,(LiPoly.Measured_SOC - SOC_Estimated*100));
legend('SOC Error');
ylabel('SOC Error [%]');
xlabel('Time[hr]');
grid minor

