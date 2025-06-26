clear
close all
clc
clearvars
% User Inputs:
% -------------------------------------------------------------------------

R_Mx = 0.;                                                              % Missile Position X-Component (ft)
R_My = 10000.;                                                          % Missile Position Y-Component (ft)
R_Tx = 40000.;                                                          % Target Position X-Component (ft)
R_Ty = 10000.;                                                          % Target Position Y-Component (ft)
V_M = 3000.;                                                            % Velocity of the missile (ft/s)
V_T = 1000.;                                                            % Velocity of the target (ft/s)
Beta = 0.;                                                              % Angle of the target velocity (degree)
XNT = 96.6;                                                             % Target maneuver
HE = -20.;                                                              % Heading error (degree)
XNP = 3.;                                                               % Effective navigation ratio or gain (Range of 3-5)
A=4; % Put break point here if necessary

% Simulink Model
% -------------------------------------------------------------------------

% Run Simulink
simOut = sim('proportional_NavSimulink');
% Store Simulink data in arrays
SimArrayT    = simOut.tout;
SimArrayR_Tx = simOut.ScopeData_RTx.signals.values;
SimArrayR_Ty = simOut.ScopeData_RTy.signals.values;
SimArrayR_Mx = simOut.ScopeData_RMx.signals.values;
SimArrayR_My = simOut.ScopeData_RMy.signals.values;
SimArrayV_Tx = simOut.ScopeData_VTx.signals.values;
SimArrayV_Ty = simOut.ScopeData_VTy.signals.values;
SimArrayV_Mx = simOut.ScopeData_VMx.signals.values;
SimArrayV_My = simOut.ScopeData_VMy.signals.values;
SimArrayV_C =  simOut.ScopeData_VC.signals.values;
SimArrayXNCG = simOut.ScopeData_XNCG.signals.values;
SimArrayR_TM = simOut.ScopeData_RTM.signals.values;
SimArrayA_Mx = simOut.ScopeData_AMx.signals.values;
SimArrayA_My = simOut.ScopeData_AMy.signals.values;
SimArrayBeta_Rad = simOut.ScopeData_BetaAngle.signals.values;
SimArrayLamda = simOut.ScopeData_Lamda.signals.values;
SimArrayLamda_D = simOut.ScopeData_LamdaD.signals.values;
%--------------------------------------------------------------------------
% Simulation settings:
%--------------------------------------------------------------------------
T = 0.;                                                                 % Initial time (s)
n = 0.;                                                                 % Length of the saved array
S = 0.;
%--------------------------------------------------------------------------
% Initial Calculations:
%--------------------------------------------------------------------------

% Convert angles degree to rad
BetaRad = Beta * pi/180;                                                % Angle of the target velocity (rad)                     
HE = HE * pi/180;                                                       % Heading error (rad)           
% Length of the Line of sight and Components (ft)
R_TMx = R_Tx - R_Mx;                                                    % Length of the Line of sight X-Component (ft)
R_TMy = R_Ty - R_My;                                                    % Length of the Line of sight Y-Component (ft)
R_TM = sqrt(R_TMx*R_TMx + R_TMy*R_TMy);                                 % Length of the Line of sight (ft)
Lamda = atan2(R_TMy , R_TMx);                                           % Line-of-sight angle (rad)
Lead = asin(V_T*sin(BetaRad+Lamda)/V_M);                                % Missile lead angle
% Target Velocity Components (ft/s)
V_Tx = -V_T * cos(BetaRad);                                             % Target velocity X-Component (ft/s)
V_Ty =  V_T * sin(BetaRad);                                             % Target velocity Y-Component (ft/s)
% Missile Velocity Components (ft/s)
V_Mx = V_M * cos(Lamda + Lead + HE);                                    % Missile velocity X-Component (ft/s)
V_My = V_M * sin(Lamda + Lead + HE);                                    % Missile velocity Y-Component (ft/s)
% Velocity of the Line of sight Components (ft/s)
V_TMx = V_Tx - V_Mx;                                                    % Velocity of the Line of sight X-Component (ft/s)
V_TMy = V_Ty - V_My;                                                    % Velocity of the Line of sight Y-Component (ft/s)
V_C = -(R_TMx * V_TMx + R_TMy * V_TMy) / R_TM;                          % Closing velocity (ft/s)

%--------------------------------------------------------------------------
% Simulation (Euler Method)
%--------------------------------------------------------------------------

while V_C >= 0.
    if R_TM < 1000
        H = 0.01; % H: Integration step size
    else
        H = 0.01;
    end
    Beta_Old = BetaRad;
    R_Tx_Old = R_Tx;    R_Ty_Old = R_Ty;
    R_Mx_Old = R_Mx;    R_My_Old = R_My;
    V_Mx_Old = V_Mx;    V_My_Old = V_My;
    % Step = 1;           Flag = 0;
    % while Step <= 1
        % if Flag == 1
            % Step = 2;
            % BetaRad = BetaRad + H*Beta_d;
            % R_Tx = R_Tx + H*V_Tx;
            % R_Ty = R_Ty + H*V_Ty;
            % R_Mx = R_Mx + H*V_Mx;
            % R_Mx = R_Mx + H*V_Mx;
            % V_Mx = V_Mx + H*A_Mx;
            % V_My = V_My + H*A_My;
                    % end
         
        R_TMx = R_Tx - R_Mx;                                            % Length of the Line of sight X-Component (ft)
        R_TMy = R_Ty - R_My;                                            % Length of the Line of sight Y-Component (ft)
        R_TM = sqrt(R_TMx*R_TMx + R_TMy*R_TMy);                         % Length of the Line of sight (ft)
        V_TMx = V_Tx - V_Mx;                                            % Velocity of the Line of sight X-Component (ft/s)
        V_TMy = V_Ty - V_My;                                            % Velocity of the Line of sight Y-Component (ft/s)
        V_C = -(R_TMx*V_TMx + R_TMy*V_TMy)/R_TM;                        % Closing velocity (ft/s)
        Lamda = atan2(R_TMy,R_TMx);                                     % Line-of-sight angle (rad)
        Lamda_d = (V_TMy*R_TMx - V_TMx*R_TMy)/(R_TM*R_TM);              % Time derivative of
        XNC = XNP*V_C*Lamda_d;                                          % Desired acceleration command (ft/s^2)
        A_Mx = -XNC*sin(Lamda);                                         % Missile acceleration X-Component
        A_My =  XNC*cos(Lamda);                                         % Missile acceleration Y-Component
        V_Tx = -V_T*cos(BetaRad);                                       % Target velocity X-Component (ft/s)
        V_Ty =  V_T*sin(BetaRad);                                       % Target velocity Y-Component (ft/s)
        Beta_d = XNT/V_T;                                               % Time derivative of the Beta angle
        % Flag = 1;
        
    % end
     
    Flag = 0;
    coeff = 1.0;
    % BetaRad = coeff*(Beta_Old + BetaRad + H*Beta_d);
    % R_Tx = coeff*(R_Tx_Old + R_Tx + H*V_Tx);
    % R_Ty = coeff*(R_Ty_Old + R_Ty + H*V_Ty);
    % R_Mx = coeff*(R_Mx_Old + R_Mx + H*V_Mx);
    % R_My = coeff*(R_My_Old + R_My + H*V_My);
    % V_Mx = coeff*(V_Mx_Old + V_Mx + H*A_Mx);
    % V_My = coeff*(V_My_Old + V_My + H*A_My);
    
    BetaRad = coeff*(Beta_Old + H*Beta_d);
    R_Tx = coeff*(R_Tx_Old + H*V_Tx);
    R_Ty = coeff*(R_Ty_Old + H*V_Ty);
    R_Mx = coeff*(R_Mx_Old + H*V_Mx);
    R_My = coeff*(R_My_Old + H*V_My);
    V_Mx = coeff*(V_Mx_Old + H*A_Mx);
    V_My = coeff*(V_My_Old + H*A_My);
    
    T = T + H; % Increase time with step size  
    S = S + H;     
    if S >=  0.009999
        S = 0.;
        n = n + 1;

        % Store MATLAB data in arrays
        ArrayT(n) = T;                                                  % Store Time
        ArrayBeta_Rad(n) = BetaRad;                                     % Store Beta Angle
        ArrayLamda(n) = Lamda;                                          % Store Lamda
        ArrayLamda_D(n) = Lamda_d;                                      % Store Lamda_d    
        ArrayR_Tx(n) = R_Tx;                                            % Store R_Tx
        ArrayR_Ty(n) = R_Ty;                                            % Store R_Ty
        ArrayR_Mx(n) = R_Mx;                                            % Store R_Mx
        ArrayR_My(n) = R_My;                                            % Store R_My
        ArrayR_TM(n) = R_TM;                                            % Store R_TM
        ArrayV_Tx(n) = V_Tx;                                            % Store V_Tx
        ArrayV_Ty(n) = V_Ty;                                            % Store V_Ty
        ArrayV_Mx(n) = V_Mx;                                            % Store V_Mx
        ArrayV_My(n) = V_My;                                            % Store V_My
        ArrayV_C(n) = V_C;                                              % Store V_C
        ArrayXNCG(n) = XNC / 32.2;                                      % Store XNCG
        ArrayA_Mx(n) = A_Mx;                                            % Store A_Mx  
        ArrayA_My(n) = A_My;                                            % Store A_My  
      
    end
end

%--------------------------------------------------------------------------
% Plotting Results
%--------------------------------------------------------------------------
%-------------------------------------------------------------------------- PLOT 1

figure;
hold on;
% Plot Simulink Target Path
plot(SimArrayR_Tx, SimArrayR_Ty, 'g-', 'DisplayName', 'Simulink Target Path');          % Green line for Simulink target path
% Plot Simulink Missile Path
plot(SimArrayR_Mx, SimArrayR_My, 'b-', 'DisplayName', 'Simulink Missile Path');         % Blue line for Simulink missile path
% Plot MATLAB Target Path
plot(ArrayR_Tx, ArrayR_Ty, 'm--', 'DisplayName', 'MATLAB Target Path ');                 % Magenta dashed line for MATLAB target path
% Plot MATLAB Missile Path
plot(ArrayR_Mx, ArrayR_My, 'k--', 'DisplayName', 'MATLAB Missile Path ');                % Black dashed line for MATLAB missile path
grid on;
title('Missile-Target Engagement Simulation: MATLAB vs. Simulink');
xlabel('Downrange (Ft)');
ylabel('Altitude (Ft)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 2

figure;
hold on;
% Plot Simulink XNCG
plot(SimArrayT, SimArrayXNCG, 'g-', 'DisplayName', 'Simulink Missile Acceleration');        % Green line for Simulink XNCG
% Plot MATLAB XNCG
plot(ArrayT, ArrayXNCG, 'r--', 'DisplayName', 'MATLAB Missile Acceleration ');              % Red dashed line for MATLAB XNCG
grid on;
title('Missile Acceleration Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('Acceleration of missile (G)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 3

figure;
hold on;
% Plot Simulink R_TM
plot(SimArrayT, SimArrayR_TM, 'g-', 'DisplayName', 'Simulink R_TM');        % Green line for Simulink R_TM
% Plot MATLAB R_TM
plot(ArrayT, ArrayR_TM, 'r--', 'DisplayName', 'MATLAB R_TM');              % Red dashed line for MATLAB R_TM
grid on;
title('R_TM Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('R_TM (ft)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 4

figure;
hold on;
% Plot Simulink V_C
plot(SimArrayT, SimArrayV_C, 'g-', 'DisplayName', 'Simulink V_C');        % Green line for Simulink V_C
% Plot MATLAB V_C
plot(ArrayT, ArrayV_C, 'r--', 'DisplayName', 'MATLAB V_C');              % Red dashed line for MATLAB V_C
grid on;
title('V_C Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('V_C (ft/s)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 5

figure;
hold on;
% Plot Simulink V_Mx
plot(SimArrayT, SimArrayV_Mx, 'g-', 'DisplayName', 'Simulink V_Mx');        % Green line for Simulink V_Mx
% Plot MATLAB V_Mx
plot(ArrayT, ArrayV_Mx, 'r--', 'DisplayName', 'MATLAB V_Mx');              % Red dashed line for MATLAB V_Mx
grid on;
title('V_Mx Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('V_Mx (ft/s)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 6
figure;
hold on;
% Plot Simulink V_My
plot(SimArrayT, SimArrayV_My, 'g-', 'DisplayName', 'Simulink V_My');        % Green line for Simulink V_My
% Plot MATLAB V_My
plot(ArrayT, ArrayV_My, 'r--', 'DisplayName', 'MATLAB V_My');              % Red dashed line for MATLAB V_My
grid on;
title('V_My Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('V_My (ft/s)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 7

figure;
hold on;
% Plot Simulink A_Mx
plot(SimArrayT, SimArrayA_Mx, 'g-', 'DisplayName', 'Simulink A_Mx');        % Green line for Simulink A_Mx
% Plot MATLAB A_Mx
plot(ArrayT, ArrayA_Mx, 'r--', 'DisplayName', 'MATLAB A_Mx');              % Red dashed line for MATLAB A_Mx
grid on;
title('A_Mx Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('A_Mx (ft/s)');
legend;
hold off;

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 8

figure;
hold on;
% Plot Simulink A_My
plot(SimArrayT, SimArrayA_My, 'g-', 'DisplayName', 'Simulink A_My');        % Green line for Simulink A_My
% Plot MATLAB A_My
plot(ArrayT, ArrayA_My, 'r--', 'DisplayName', 'MATLAB A_My');              % Red dashed line for MATLAB A_My
grid on;
title('A_My Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('A_My (ft/s)');
legend;
hold off;
