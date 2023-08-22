%State of Charge Estimation Function 
% based on Extended Kalman Filter
function [SOC_Estimated, Vt_Estimated, Vt_Error] = EKF_function(Current, Vt_Actual, Temperature)
load 'BatteryModel.mat'; % Load the battery parameters 
load 'SOC-OCV.mat'; % Load the SOC-OCV curve
SOC_Init    = 1; % intial SOC
X           = [SOC_Init; 0; 0]; % state space x parameter intializations
DeltaT      = 1; % sample time in seconds
Qn_rated    = 4.81 * 3600; % Ah to Amp-seconds

% initialize scatteredInterpolant functions for battery parameters and SOC-OCV curve
% this function also allows for extrapolation
F_R0    = scatteredInterpolant(param.T,param.SOC,param.R0);
F_R1    = scatteredInterpolant(param.T,param.SOC,param.R1);
F_R2    = scatteredInterpolant(param.T,param.SOC,param.R2);
F_C1    = scatteredInterpolant(param.T,param.SOC,param.C1);
F_C2    = scatteredInterpolant(param.T,param.SOC,param.C2);
% F_OCV   = scatteredInterpolant(param.T,param.SOC,param.OCV);  
% OCV can be extrapolated using the same method or through the polyfit function
SOCOCV  = polyfit(SOC_OCV.SOC,SOC_OCV.OCV,11); % calculate 11th order polynomial for the SOC-OCV curve 
dSOCOCV = polyder(SOCOCV); % derivative of SOC-OCV curve for matrix C
n_x   = size(X,1);
R_x   = 2.5e-1;
P_x   = [0.025 0 0;
         0 0.01 0;
         0 0 0.01];
Q_x   = [1.0e-5 0 0;
         0 1.0e-5 0;
         0 0 1.0e-6];
%Initialize the output vectors and set the length of the for loop
SOC_Estimated   = [];
Vt_Estimated    = [];
Vt_Error        = [];
ik              = length(Current);
% now kalmann filter algorithm starts
for k=1:1:ik
    T           = Temperature(k); % C
    U           = Current(k); % A
    SOC         = X(1);
    V1          = X(2);
    V2          = X(3);
    
    % Evaluate the battery parameter scatteredInterpolant 
    % functions for the current temperature & SOC
    R0     = F_R0(T,SOC);
    R1     = F_R1(T,SOC);
    R2     = F_R2(T,SOC);
    C1     = F_C1(T,SOC);
    C2     = F_C2(T,SOC);
    % OCV    = F_OCV(T,SOC);
    % OCV    = pchip(param.SOC,param.OCV,SOC); % pchip sample for unknown or single temperature
    
    OCV = polyval(SOCOCV,SOC); % calculate the values of OCV at the given SOC, using the polynomial SOCOCV
    Tau_1       = C1 * R1;
    Tau_2       = C2 * R2;
    
    a1 = exp(-DeltaT/Tau_1);
    a2 = exp(-DeltaT/Tau_2);
    
    b1 = R1 * (1 - exp(-DeltaT/Tau_1));
    b2 = R2 * (1 - exp(-DeltaT/Tau_2)); 
    TerminalVoltage = OCV - R0*U - V1 - V2;

    if U > 0
        eta = 1; % eta for discharging
    elseif U <= 0 
        eta = 1; % eta for charging
    end


    dOCV = polyval(dSOCOCV, SOC);
    C_x    = [dOCV -1 -1];
    Error_x   = Vt_Actual(k) - TerminalVoltage;
    Vt_Estimated    = [Vt_Estimated;TerminalVoltage];
    SOC_Estimated   = [SOC_Estimated;X(1)];
    Vt_Error        = [Vt_Error;Error_x];

    %EKF
    A   = [1 0  0;
           0 a1 0;
           0 0  a2];
    B   = [-(eta * DeltaT/Qn_rated); b1; b2];
    X   = (A * X) + (B * U);
    P_x = (A * P_x * A') + Q_x;
    KalmanGain_x = (P_x) * (C_x') * (inv((C_x * P_x * C_x') + (R_x)));
    X            = X + (KalmanGain_x * Error_x);
    P_x          = (eye(n_x,n_x) - (KalmanGain_x * C_x)) * P_x;
    
end
