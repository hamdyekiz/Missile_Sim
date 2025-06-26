tic;

clear
close all
clc
clearvars

% User-defined constants (up to 6)
user_index = 4; 

%=============================================================================================
% Useful Constants

d2r = pi/180;                             % Conversion Deg to Rad
r2d = 180/pi;                             % Conversion Rad to Deg
m2ft = 3.28084;                           % meter to feet
kg2slug = 0.0685218;                      % kg to slug
kg2lb = 2.20462;                          % kg to pounds
kgm2_to_lbft2 = 23.73036;                 % kg·m^2 to lb·ft^2

ft2m = 0.3048;                            % feet to meters
slug2kg = 14.5939;                        % slug to kilograms
lb2kg = 0.453592;                         % pounds to kilograms
lbft2_to_kgm2 = 0.0421401;                % lb·ft^2 to kg·m^2
g = 9.81;                                 % Gravitational constant (m/s^2)

%=============================================================================================

%=============================================================================================
% Blakelock's List of Missile Parameters
ALT_array = [159, 464, 2400, 5819, 8524, 8927];
V_M_array = [387.67, 652.37, 829.8, 1139.62, 884.12, 851.87];

ALT = ALT_array(user_index);
V_M_b = V_M_array(user_index);

%=============================================================================================

%=============================================================================================
% User Inputs:
R_Mx = 0;                            % Missile Position X-Component (m)
R_My = ALT;                               % Missile Position Y-Component (m) 
R_Tx = 6000.;                       % Target Position X-Component (m)
R_Ty = 8000.;                       % Target Position Y-Component (m)
V_M = V_M_b;                              % Velocity of the missile (m/s)
V_T = 300;                           % Velocity of the target (m/s)
Beta = -10.;                                % Angle of the target velocity (degree)
XNT = -3*g;                                  % Target maneuver
HE = -5.;                                % Heading error (degree)
XNP = 3.;                                 % Effective navigation ratio or gain (Range of 3-5)

%============================================================================================= 
% Create Main folder to store simulation results
mainFolder = 'guidanceWithoutControlSystem_simulationResults';
if ~exist(mainFolder, 'dir')
    mkdir(mainFolder);
end

% Store simulation parameters in a text file
simulationParameters = {
    sprintf('R_Mx = %.9f (Missile Position X-Component, m)', R_Mx);
    sprintf('R_My = %.9f (Missile Position Y-Component, m)', R_My);
    sprintf('R_Tx = %.9f (Target Position X-Component, m)', R_Tx);
    sprintf('R_Ty = %.9f (Target Position Y-Component, m)', R_Ty);
    sprintf('V_M = %.9f (Velocity of the missile, m/s)', V_M);
    sprintf('V_T = %.9f (Velocity of the target, m/s)', V_T);
    sprintf('Beta = %.9f (Angle of the target velocity, degree)', Beta);
    sprintf('XNT = %.9f (Target maneuver, m/s^2)', XNT);
    sprintf('HE = %.9f (Heading error, degree)', HE);
    sprintf('XNP = %.9f (Effective navigation ratio or gain)', XNP);
};

% File path for simulation parameters
parametersFile = fullfile(mainFolder, 'simulationParameters.txt');

% Write parameters to the text file
fileID = fopen(parametersFile, 'w');
for i = 1:length(simulationParameters)
    fprintf(fileID, '%s\n', simulationParameters{i});
end
fclose(fileID);

%============================================================================================= 

%=============================================================================================
% Simulink Model

% Run Simulink
simOut = sim('proportionalNav_withoutControlSystem');

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

%=============================================================================================

%=============================================================================================
% Initial Calculations:

% Convert angles degree to rad
BetaRad = d2r* Beta;                      % Angle of the target velocity (rad)                     
HE = d2r* HE;                             % Heading error (rad)           

% Length of the Line of sight and Components (m)
R_TMx = R_Tx - R_Mx;                      % Length of the Line of sight X-Component (m)
R_TMy = R_Ty - R_My;                      % Length of the Line of sight Y-Component (m)
R_TM = sqrt(R_TMx*R_TMx + R_TMy*R_TMy);   % Length of the Line of sight (m)

Lamda = atan2(R_TMy , R_TMx);             % Line-of-sight angle (rad)
Lead = asin(V_T*sin(BetaRad+Lamda)/V_M);  % Missile lead angle

% Target Velocity Components (m/s)
V_Tx = -V_T * cos(BetaRad);               % Target velocity X-Component (m/s)
V_Ty =  V_T * sin(BetaRad);               % Target velocity Y-Component (m/s)

% Missile Velocity Components (m/s)
V_Mx = V_M * cos(Lamda + Lead + HE);      % Missile velocity X-Component (m/s)
V_My = V_M * sin(Lamda + Lead + HE);      % Missile velocity Y-Component (m/s)

% Velocity of the Line of sight Components (m/s)
V_TMx = V_Tx - V_Mx;                      % Velocity of the Line of sight X-Component (m/s)
V_TMy = V_Ty - V_My;                      % Velocity of the Line of sight Y-Component (m/s)

V_C = -(R_TMx * V_TMx + R_TMy * V_TMy) / R_TM;                     % Closing velocity (m/s)

%============================================================================================= 

%============================================================================================= 
T = 0.;                                   % Initial time (s)
n = 0.;                                   % Length of the saved array

% Integration using "Runge Kutta 4th Order Method"     
while V_C >= 0. 
    if R_TM < 1000 
        H = 0.001;                        % Integration step size (s)
    else
        H = 0.001;
    end

    X_s = [BetaRad, R_Tx, R_Ty, R_Mx, R_My, V_Mx, V_My];
    X_b = X_s;
    L = length(X_b);

    BetaRad = X_s(1);
    R_Tx = X_s(2);
    R_Ty = X_s(3);
    R_Mx = X_s(4);
    R_My = X_s(5);
    V_Mx = X_s(6);
    V_My = X_s(7);

    T = T + H;                               % Increase time with step size  
     
    R_TMx = R_Tx - R_Mx;                     % Length of the Line of sight X-Component (m) 
    R_TMy = R_Ty - R_My;                     % Length of the Line of sight Y-Component (m) 
    R_TM = sqrt(R_TMx*R_TMx + R_TMy*R_TMy);  % Length of the Line of sight (m) 
    V_TMx = V_Tx - V_Mx;                     % Velocity of the Line of sight X-Component (m/s) 
    V_TMy = V_Ty - V_My;                     % Velocity of the Line of sight Y-Component (m/s) 
    V_C = -(R_TMx*V_TMx + R_TMy*V_TMy)/R_TM; % Closing velocity (m/s) 
    Lamda = atan2(R_TMy,R_TMx);              % Line-of-sight angle (rad) 
    Lamda_d = (V_TMy*R_TMx - V_TMx*R_TMy)/(R_TM*R_TM);% Time derivative of line-of-sight angle
    XNC = XNP*V_C*Lamda_d;                   % Desired acceleration command (m/s^2) 
    A_Mx = -XNC*sin(Lamda);                  % Missile acceleration X-Component (m/s^2)
    A_My =  XNC*cos(Lamda);                  % Missile acceleration Y-Component (m/s^2)
    V_Tx = -V_T*cos(BetaRad);                % Target velocity X-Component (m/s) 
    V_Ty =  V_T*sin(BetaRad);                % Target velocity Y-Component (m/s) 
    Beta_d = XNT/V_T;                        % Time derivative of the Beta angle 
     
%=============================================================================================
   
    % Calculate K1
    K1 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My];
    
    % Calculate K2
    for i = 1:L
        X_t(i) = X_b(i) + 0.5 * H * K1(i);
    end
    K2 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My];
    
    % Calculate K3
    for i = 1:L
        X_t(i) = X_b(i) + 0.5 * H * K2(i);
    end
    K3 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My];
    
    % Calculate K4
    for i = 1:L
        X_t(i) = X_b(i) + H * K3(i);
    end
    K4 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My];

    % Update state variables using weighted sum of K1, K2, K3, and K4
    for i = 1:L
        X_s(i) = X_b(i) + (H / 6) * (K1(i) + 2 * K2(i) + 2 * K3(i) + K4(i));
    end

    BetaRad = X_s(1);
    R_Tx = X_s(2);
    R_Ty = X_s(3);
    R_Mx = X_s(4);
    R_My = X_s(5);
    V_Mx = X_s(6);
    V_My = X_s(7);

    n = n + 1; 
    
    % Store MATLAB data in arrays
    ArrayT(n) = T;                           % Store Time
    ArrayBeta_Rad(n) = BetaRad;              % Store Beta Angle
    ArrayLamda(n) = Lamda;                   % Store Lamda
    ArrayLamda_D(n) = Lamda_d;               % Store Lamda_d    
    ArrayR_Tx(n) = R_Tx;                     % Store R_Tx
    ArrayR_Ty(n) = R_Ty;                     % Store R_Ty 
    ArrayR_Mx(n) = R_Mx;                     % Store R_Mx 
    ArrayR_My(n) = R_My;                     % Store R_My 
    ArrayR_TM(n) = R_TM;                     % Store R_TM
    ArrayV_Tx(n) = V_Tx;                     % Store V_Tx
    ArrayV_Ty(n) = V_Ty;                     % Store V_Ty
    ArrayV_Mx(n) = V_Mx;                     % Store V_Mx
    ArrayV_My(n) = V_My;                     % Store V_My
    ArrayV_C(n) = V_C;                       % Store V_C
    ArrayXNLG(n) = XNC /g;                   % Store XNL
    ArrayA_Mx(n) = A_Mx;                     % Store A_Mx  
    ArrayA_My(n) = A_My;                     % Store A_My 
    
end 

%============================================================================================= 
% Write all results into text files 

% Create Subfolders for MATLAB and Simulink results
matlabFolder = fullfile(mainFolder, 'MATLAB_Results');
simulinkFolder = fullfile(mainFolder, 'Simulink_Results');

if ~exist(matlabFolder, 'dir')
    mkdir(matlabFolder);
end
if ~exist(simulinkFolder, 'dir')
    mkdir(simulinkFolder);
end

% MATLAB Arrays and corresponding file names
matlabArrays = {ArrayT, ArrayBeta_Rad, ArrayLamda, ArrayLamda_D, ArrayR_Tx, ...
                ArrayR_Ty, ArrayR_Mx, ArrayR_My, ArrayR_TM, ArrayV_Tx, ...
                ArrayV_Ty, ArrayV_Mx, ArrayV_My, ArrayV_C, ArrayXNLG, ...
                ArrayA_Mx, ArrayA_My};

matlabFileNames = {'Time.txt', 'Beta_Rad.txt', 'Lamda.txt', 'Lamda_D.txt', 'R_Tx.txt', ...
                   'R_Ty.txt', 'R_Mx.txt', 'R_My.txt', 'R_TM.txt', 'V_Tx.txt', ...
                   'V_Ty.txt', 'V_Mx.txt', 'V_My.txt', 'V_C.txt', 'XNC.txt', ...
                   'A_Mx.txt', 'A_My.txt'};

% Simulink Arrays and corresponding file names
simulinkArrays = {SimArrayT, SimArrayR_Tx, SimArrayR_Ty, SimArrayR_Mx, SimArrayR_My, ...
                  SimArrayV_Tx, SimArrayV_Ty, SimArrayV_Mx, SimArrayV_My, SimArrayV_C, ...
                  SimArrayXNCG, SimArrayR_TM, SimArrayA_Mx, SimArrayA_My, ...
                  SimArrayBeta_Rad, SimArrayLamda, SimArrayLamda_D};

simulinkFileNames = {'Time.txt', 'R_Tx.txt', 'R_Ty.txt', 'R_Mx.txt', 'R_My.txt', ...
                     'V_Tx.txt', 'V_Ty.txt', 'V_Mx.txt', 'V_My.txt', 'V_C.txt', ...
                     'XNCG.txt', 'R_TM.txt', 'A_Mx.txt', 'A_My.txt', ...
                     'BetaAngle.txt', 'Lamda.txt', 'LamdaD.txt'};

% Write MATLAB results to MATLAB_Results folder
for i = 1:length(matlabArrays)
    filePath = fullfile(matlabFolder, matlabFileNames{i});
    writematrix(matlabArrays{i}', filePath, 'Delimiter', '\t'); 
end

% Write Simulink results to Simulink_Results folder
for i = 1:length(simulinkArrays)
    filePath = fullfile(simulinkFolder, simulinkFileNames{i});
    writematrix(simulinkArrays{i}(:), filePath, 'Delimiter', '\t'); 
end

%============================================================================================= 

%=============================================================================================
% Plotting Results

% PLOT 1: Target and Missile Path (MATLAB vs Simulink)
figure;
hold on;

% Plot Simulink Target Path
plot(SimArrayR_Tx, SimArrayR_Ty, 'g-', 'DisplayName', 'Simulink Target Path'); 
% Plot Simulink Missile Path
plot(SimArrayR_Mx, SimArrayR_My, 'b-', 'DisplayName', 'Simulink Missile Path');    

% Plot MATLAB Target Path
plot(ArrayR_Tx, ArrayR_Ty, 'm--', 'DisplayName', 'MATLAB Target Path ');
% Plot MATLAB Missile Path
plot(ArrayR_Mx, ArrayR_My, 'k--', 'DisplayName', 'MATLAB Missile Path ');   

grid on;
title('Missile-Target Engagement Simulation: MATLAB vs. Simulink');
xlabel('Downrange (m)');
ylabel('Altitude (m)');
legend;
hold off;
%============================================================================================= 
% PLOT 2: Target and Missile Acceleration (MATLAB vs Simulink)

figure;
hold on;

plot(SimArrayT(1:end-1), SimArrayXNCG(1:end-1), 'g-', 'DisplayName', 'Simulink Missile Acceleration');    
plot(ArrayT(1:end-1), ArrayXNLG(1:end-1), 'r--', 'DisplayName', 'MATLAB Missile Acceleration');         

grid on;
title('Missile Acceleration Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('Acceleration of missile (G)');
legend;
hold off;

%============================================================================================= 
% PLOT 3: Line of Sight Length (MATLAB vs Simulink)

figure;
hold on;

% Plot Simulink R_TM
plot(SimArrayT, SimArrayR_TM, 'g-', 'DisplayName', 'Simulink R_TM');      

% Plot MATLAB R_TM
plot(ArrayT, ArrayR_TM, 'r--', 'DisplayName', 'MATLAB R_TM');          

grid on;
title('R_TM Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('R_TM (ft)');
legend;
hold off;
%============================================================================================= 
% PLOT 4: Closing Velocity (MATLAB vs Simulink)

figure;
hold on;

plot(SimArrayT(1:end-1), SimArrayV_C(1:end-1), 'g-', 'DisplayName', 'Simulink V_C');      
plot(ArrayT(1:end-1), ArrayV_C(1:end-1), 'r--', 'DisplayName', 'MATLAB V_C');             

grid on;
title('V_C Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('V_C (ft/s)');
legend;
hold off;

%============================================================================================= 
% PLOT 5: Target and Missile X-Acceleration (MATLAB vs Simulink)

figure;
hold on;

% Plot Simulink A_Mx
plot(SimArrayT, SimArrayA_Mx, 'g-', 'DisplayName', 'Simulink A_Mx');        

% Plot MATLAB A_Mx
plot(ArrayT, ArrayA_Mx, 'r--', 'DisplayName', 'MATLAB A_Mx');             

grid on;
title('A_Mx Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('A_Mx (ft/s)');
legend;
hold off;
%=============================================================================================  
% PLOT 6: Target and Missile Y-Acceleration (MATLAB vs Simulink)

figure;
hold on;

% Plot Simulink A_My
plot(SimArrayT, SimArrayA_My, 'g-', 'DisplayName', 'Simulink A_My');       

% Plot MATLAB A_My
plot(ArrayT, ArrayA_My, 'r--', 'DisplayName', 'MATLAB A_My');             

grid on;
title('A_My Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('A_My (ft/s)');
legend;
hold off;
%=============================================================================================  

%--------------------------------------------------------------------------  
%-------------------------------------------------------------------------- PLOT 7 

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

% =============================================================================================   
%-------------------------------------------------------------------------- PLOT 8 

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

%-------------------------------------------------------------------------- PLOT 9 

figure;
hold on;

% Plot Simulink Target Path
plot(SimArrayV_Tx, SimArrayV_Ty, 'g-', 'DisplayName', 'Simulink Target Velocity');          % Green line for Simulink target path
% Plot Simulink Missile Path
plot(SimArrayV_Mx, SimArrayV_My, 'b-', 'DisplayName', 'Simulink Missile Velocity');         % Blue line for Simulink missile path

% Plot MATLAB Target Path
plot(ArrayV_Tx, ArrayV_Ty, 'm--', 'DisplayName', 'MATLAB Target Velocity ');                 % Magenta dashed line for MATLAB target path
% Plot MATLAB Missile Path
plot(ArrayV_Mx, ArrayV_My, 'k--', 'DisplayName', 'MATLAB Missile Velocity ');                % Black dashed line for MATLAB missile path

grid on;
title('Missile-Target Velocity Simulation: MATLAB vs. Simulink');
xlabel('Downrange (Ft)');
ylabel('Altitude (Ft)');
legend;
hold off;

%-------------------------------------------------------------------------- PLOT 10 

figure;
hold on;

% Plot Simulink Lamda_D
plot(SimArrayT, SimArrayLamda_D, 'g-', 'DisplayName', 'Simulink Lamda_D');          % Green line for Simulink Lamda_D

% Plot MATLAB Lamda_D
plot(ArrayT, ArrayLamda_D, 'm--', 'DisplayName', 'MATLAB Lamda_D ');                 % Magenta dashed line for MATLAB Lamda_D

grid on;
title('Lamda_D: MATLAB vs. Simulink');
xlabel('Downrange (Ft)');
ylabel('Altitude (Ft)');
legend;
hold off;

%-------------------------------------------------------------------------- PLOT 11 

figure;
hold on;

% Plot Simulink Lamda
plot(SimArrayT, SimArrayLamda, 'g-', 'DisplayName', 'Simulink Lamda');          % Green line for Simulink Lamda

% Plot MATLAB Lamda
plot(ArrayT, ArrayLamda, 'm--', 'DisplayName', 'MATLAB Lamda ');                 % Magenta dashed line for MATLAB Lamda

grid on;
title('Lamda: MATLAB vs. Simulink');
xlabel('Downrange (Ft)');
ylabel('Altitude (Ft)');
legend;
hold off;
