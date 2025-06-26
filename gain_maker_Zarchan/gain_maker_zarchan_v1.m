% --- Constants ---
DIAM = 0.3048;     % Missile diameter (m)
SREF = 0.0730;     % Reference area (m^2)
g = 9.81;          % Acceleration due to gravity (m/s^2)
WCR = 50;          % Crossover Frequency (rad/s)
ZETA = 0.7;        % Actuator damping
TAU = 0.3;         % Time constant (s)
r2d = 180 / pi;    % Degrees to radians conversion

% --- Read dynamic data from CSV file ---
dataTable = readtable('dynamic_data.csv');

% Convert table to a struct for easier access
data.t = dataTable.t;
data.alpha = dataTable.alpha;
data.Q = dataTable.Q;
data.h = dataTable.h;
data.V = dataTable.V;
data.mach = dataTable.MACH;
data.mass = dataTable.mass;
data.Iyy = dataTable.Iyy;
data.CM_alpha = dataTable.CMA;
data.CN_alpha = dataTable.CNA;
data.CM_delta = dataTable.CMD/0.0174533;
data.CN_delta = dataTable.CND/0.0174533;

data_count = length(data.t)
% --- Compute gains ---
[XKDC_out, XKA_out, WI_out, XKR_out] = calculateAutopilotGains(data, DIAM, SREF, g, WCR, ZETA, TAU, r2d);

% % --- Remove Outlier Data ---
% mask_XKDC = isoutlier(XKDC_out, "quartiles");
% mask_XKA  = isoutlier(XKA_out, "quartiles");
% mask_WI   = isoutlier(WI_out, "quartiles");
% mask_XKR  = isoutlier(XKR_out, "quartiles");
% % Combine masks so that any sample flagged by any one of the four gains is removed.
% combinedMask = mask_XKDC | mask_XKA | mask_WI | mask_XKR;
% 
% % Apply the “keep” mask (i.e. ~combinedMask) to every vector/table:
% keepIdx = ~combinedMask;
% 
% % Subset time and Q from dataTable
dataTable_t = dataTable.t;
dataTable_Q = dataTable.Q;
% 
% % Subset the four gain vectors
% XKDC_out = XKDC_out(keepIdx);
% XKA_out  = XKA_out(keepIdx);
% WI_out   = WI_out(keepIdx);
% XKR_out  = XKR_out(keepIdx);

% --- Create a new table with only the desired columns ---
autopilotGainsTable = table();
autopilotGainsTable.t    = dataTable_t;
autopilotGainsTable.Q    = dataTable_Q;
autopilotGainsTable.XKDC = XKDC_out;
autopilotGainsTable.XKA  = XKA_out;
autopilotGainsTable.WI   = WI_out;
autopilotGainsTable.XKR  = XKR_out;

% --- Save the results to CSV ---
writetable(autopilotGainsTable, 'autopilot_gains.csv');
fprintf('Autopilot gains computed and saved to autopilot_gains.csv\n');


% --- Function to calculate autopilot gains ---
function [XKDC_out, XKA_out, WI_out, XKR_out] = calculateAutopilotGains(data, DIAM, SREF, g, WCR, ZETA, TAU, r2d)
    num_points = length(data.t);
    XKDC_out = zeros(num_points, 1);
    XKA_out = zeros(num_points, 1);
    WI_out = zeros(num_points, 1);
    XKR_out = zeros(num_points, 1);

    for i = 1:num_points
        Q = data.Q(i);
        CM_alpha = data.CM_alpha(i);
        CM_delta = data.CM_delta(i);
        CN_alpha = data.CN_alpha(i);
        CN_delta = data.CN_delta(i);
        mass = data.mass(i);
        V_M = data.V(i);
        XIYY = data.Iyy(i);

        WGT = mass;

        % Calculate intermediate variables
        XMA = -Q * SREF * DIAM * CM_alpha / XIYY;
        XMD = -Q * SREF * DIAM * CM_delta / XIYY;
        ZA  =  Q * SREF * CN_alpha / (WGT * V_M);
        ZD  =  Q * SREF * CN_delta / (WGT * V_M);

        WZ = sqrt((XMA*ZD-ZA*XMD)/ZD);            % ω_z:  Airframe Zero (rad/s)
        WAF = sqrt(-XMA);                         % ω_AF: Airframe Natural Frequency (rad/s)
        ZAF = 0.5*WAF*ZA/XMA;                     % ξ_AF: Airframe Damping
        XK1 = -V_M*(XMA*ZD-XMD*ZA)/(XMA); % Gain K1: Aerodynamic acceleration gain (g/deg)
        XK2 = XK1;                                % Gain K2: Aerodynamic acceleration gain (g/deg)
        TA = XMD/(XMA*ZD-XMD*ZA);                 % T_∝: Turning rate time constant (s)
        XK3 = -(XMA*ZD-XMD*ZA)/(XMA);             % Gain K3: Aerodynamic body rate gain (1/s)
        
        W = (TAU*WCR*(1+2.*ZAF*WAF/WCR)-1)/(2*ZETA*TAU); % Autopilot design parameter  (rad/s)
        W0 = W/sqrt(TAU*WCR);                            % Normalized frequency for autopilot 
        Z0 = 0.5*W0*(2*ZETA/W+TAU-WAF^2/(W0*W0*WCR));    % Damping for autopilot 
        XKC = (-W0^2/WZ^2-1.+2.*Z0*W0*TA)/(1.-2.*Z0*W0*TA+W0*W0*TA*TA);  % Feedback gain for autopilot 
        XKA = XK3/(XK1*XKC);                      % Accelerometer autopilot gain
        XK0 = -W*W/(TAU*WAF*WAF);                 % Open-loop gain for autopilot 
        XK = XK0/(XK1*(1+XKC));                   % Closed-loop gain for autopilot 
        WI = XKC*TA*W0*W0/(1+XKC+W0^2/WZ^2);      % Integrator frequency for autopilot
        XKR = XK/(XKA*WI);                        % Rate gyro gain for autopilot
        XKDC = 1 + 1/ (XKA*V_M);             % Steady-state gain for autopilot    


        

        XKDC_out(i) = XKDC;
        XKA_out(i) = XKA;
        WI_out(i) = WI;
        XKR_out(i) = XKR;
    end
end
