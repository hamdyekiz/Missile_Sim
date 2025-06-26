clear
close all
clc
clearvars

% User-defined constants (up to 6)
user_index = 4; 

%=============================================================================================
% Useful Constants

d2r = pi/180;                                   % Conversion Deg to Rad
r2d = 180/pi;                                   % Conversion Rad to Deg
m2ft = 3.28084;                                 % meter to feet
kg2slug = 0.0685218;                            % kg to slug
kg2lb = 2.20462;                                % kg to pounds
kgm2_to_lbft2 = 23.73036;                       % kg·m^2 to lb·ft^2

ft2m = 0.3048;                                  % feet to meters
slug2kg = 14.5939;                              % slug to kilograms
lb2kg = 0.453592;                               % pounds to kilograms
lbft2_to_kgm2 = 0.0421401;                      % lb·ft^2 to kg·m^2
g = 9.81;                                       % Gravitational constant (m/s^2)

%=============================================================================================

%=============================================================================================
% Blakelock's List of Missile Parameters for Dynamic Analysis
ALT_array = [159, 464, 2400, 5819, 8524, 8927];
V_M_array = [387.67, 652.37, 829.8, 1139.62, 884.12, 851.87];
XMACH_array = [1.14, 1.928, 2.508, 3.59, 2.89, 2.802];
WGT_array = [908.8, 798, 623, 423, 423, 423];
XIYY_array = [2334.4, 2050, 1767.69, 1445, 1445, 1445];
CMA_array = [-123.8, -61.2, -36.78, -9.04, -27.07, -29.3]; 
CNA_array = [19.22, 15.25, 13.1, 10.78, 11.39, 11.79]; 
CMD_array = [-120.88, -97.98, -78.99, -58.79, -70.24, -72.05]; 
CND_array = [8.3, 6.72, 5.41, 4.03, 4.82, 4.94]; 
XCG_array = [-3.578, -3.49, -3.336, -3.16, -3.16, -3.16];

tf1 = tf([-106.47, -106.47 * 0.418], [1, 0.644, 86.4]);
tf2 = tf([-279.61, -279.61 * 0.775], [1, 0.95, 116.87]);
tf3 = tf([-369.4, -369.4 * 0.94], [1, 1.098, 126.4]);
tf4 = tf([-469.6, -469.6 * 1.2], [1, 1.27, 72.25]);
tf5 = tf([-247.7, -247.7 * 0.64], [1, 0.764, 95.46]);
tf6 = tf([-224.75, -224.75 * 0.603], [1, 0.726, 91.4]);

blakelock_tf_array = [tf1, tf2, tf3, tf4, tf5, tf6];

% Air Density Table Values (interpolated) (kg/m^3)
% from Anderson's Fundamentals of Aerodynamics Book
RHO_array = [1.209642, 1.171404, 0.96673, 0.673454, 0.4943484, 0.4711699]; 
RHO = RHO_array(user_index);                                            

ALT = ALT_array(user_index);
V_M = V_M_array(user_index);
XMACH = XMACH_array(user_index);
WGT = WGT_array(user_index);
XIYY = XIYY_array(user_index);
CMA = CMA_array(user_index);
CNA = CNA_array(user_index);
CMD = CMD_array(user_index);
CND = CND_array(user_index);
XCG = XCG_array(user_index);

%=============================================================================================

%=============================================================================================
% Calculations

n = 0;                                        % Array number initialize                                                                      

DEL = 0;                                      % Control surface deflection (degree)
DIAM = 0.203;                                 % Missile diameter (m)
DEL_Rad = 5/r2d;                              % Control surface deflection (radian)
SREF = 0.13;                                  % Reference area (m^2)
Q = 0.5*RHO*V_M*V_M;                          % Dynamic pressure 
XMA = Q*SREF*DIAM*CMA/XIYY;                   % M_∝ (1/s^2)
XMD = Q*SREF*DIAM*CMD/XIYY;                   % M_δ (1/s^2)
ZA = -Q*SREF*CNA/((WGT)*V_M);                 % Z_∝ (1/s)
ZD = -Q*SREF*CND/((WGT)*V_M);                 % Z_δ (1/s)
WZ = sqrt((XMA*ZD-ZA*XMD)/ZD);                % ω_z: Airframe Zero (rad/s)
WAF = sqrt(-XMA);                             % ω_AF: Airframe Natural Frequency (rad/s)
ZAF = 0.5*WAF*ZA/XMA;                         % ξ_AF: Airframe Damping
XK1 = -V_M*(XMA*ZD-XMD*ZA)/((g*r2d)*XMA);     % Gain K1: Aerodynamic acceleration gain (g/deg)
XK2 = XK1;                                    % Gain K2: Aerodynamic acceleration gain (g/deg)
TA = XMD/(XMA*ZD-XMD*ZA);                     % T_∝: Turning rate time constant (s)
XK3 = -(XMA*ZD-XMD*ZA)/(XMA);                 % Gain K3: Aerodynamic body rate gain (1/s)

E = 0.;                                       % e from chain rule 
ED = 0.;                                      % e_dot
T = 0;                                        % Time
H = 0.0025;                                   % Time step (s)      

% Integrate using "Euler Integration Method"
while T < 10.99999

    E_OLD = E;                                                            
    ED_OLD = ED;                                                          

    EDD = WAF*WAF*(DEL-E-2.*ZAF*ED/WAF);      % e_ddot
    XNL = XK1*(E-(EDD/WZ^2));                 % Missile acceleration (m/s^2)
    THD = XK3*(E+TA*ED);                      % Derivative of the theta angle


% Save T, XNL in an array   
    n = n+1;
    ArrayT(n) = T;
    ArrayXNL(n) = XNL;

    coeff = 1.0;   
    E = coeff*(E_OLD + H*ED);
    ED = coeff*(ED_OLD + H*EDD);
    T = T+H;

end

%=============================================================================================

%=============================================================================================
% Plotting

% Plot Step Response
figure

zarchan_tf = tf([XK3*TA XK3],[1/(WAF*WAF) 2*ZAF/WAF 1]);
step(zarchan_tf,'g-');
hold on
blakelock_tf = blakelock_tf_array(user_index);
step(blakelock_tf, 'r--');
hold off

legend('Zarchan TF', 'Blakelock TF'); 
title(sprintf(' Open Loop Step Response: Zarchan vs. Blakelock (Transfer Function: %d)', user_index));
grid on; 


% Plot Bode plot 
figure;
bode(zarchan_tf, 'g-', blakelock_tf, 'r--');
grid on;
legend('Zarchan TF', 'Blakelock TF');
title(sprintf('Bode Plot: Zarchan vs. Blakelock (Transfer Function: %d)', user_index));

%=============================================================================================
% Calculate poles, natural frequency, damping ratio etc. 
% For both Zarchan and Blakelock
zarchan_poles = pole(zarchan_tf)
zarchan_wn = abs(zarchan_poles)
zarchan_zeta = -real(zarchan_poles) ./ zarchan_wn

blakelock_poles = pole(blakelock_tf)
blakelock_wn = abs(blakelock_poles)
blakelock_zeta = -real(blakelock_poles) ./ blakelock_wn

%=============================================================================================

