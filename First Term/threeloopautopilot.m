clear
close all
clc
clearvars

% User-defined constants
user_index = 4; 
%==================================================================
% Useful Constants
%==================================================================

d2r     = pi/180;                 % Conversion Deg to Rad
g       = 9.81;                   % Gravity [m/s/s]
m2ft    = 3.28084;                % meter to feet
kg2slug = 0.0685218;              % kg to slug
kg2lb = 2.20462;                  % kg to pounds
kgm2_to_lbft2 = 23.73036;         % kg·m^2 to lb·ft^2

% Blakelock's List of Missile Parameters for Dynamic Analysis
ALT_array = m2ft*[159, 464, 2400, 5819, 8524, 8927];
V_M_array = m2ft*[387.67, 652.37, 829.8, 1139.62, 884.12, 851.87];
XMACH_array = [1.14, 1.928, 2.508, 3.59, 2.89, 2.802];
WGT_array = kg2lb*[908.8, 798, 623, 423, 423, 423];
XIYY_array = kgm2_to_lbft2*[2334.4, 2050, 1767.69, 1445, 1445, 1445];
CMA_array = [-123.8, -61.2, -36.78, -9.04, -27.07, -29.3]; 
CNA_array = [19.22, 15.25, 13.1, 10.78, 11.39, 11.79]; 
CMD_array = [-120.88, -97.98, -78.99, -58.79, -70.24, -72.05]; 
CND_array = [8.3, 6.72, 5.41, 4.03, 4.82, 4.94]; 
XCG_array = m2ft*[-3.578, -3.49, -3.336, -3.16, -3.16, -3.16];

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

n = 0;
% V_M = 3000;                                                                % Missile velocity (ft/s)
% DEL = 5;                                                                   % Control surface deflection (degree)
% ALT = 5000.;                                                               % Altitude (ft)   
% A = 1000.;                                                                 % Speed of sound (ft/s)
% DIAM = 1.;                                                                   % Missile diameter (ft)
DIAM = m2ft*0.203;                                                                   % Missile diameter (ft)
FR = 3.;                                                                     % Radome length (ft)
XL = 20.;                                                                    % Missile length (ft)
CTW = 0.;                                                                    % Wing tip chord length (ft)
CRW = 6.;                                                                    % Wing root chord length (ft)
HW = 2.;                                                                     % Wing height (ft)
CTT = 0.;                                                                    % Tail tip chord length (ft) 
CRT = 2.;                                                                    % Tail root chord length (ft)
HT = 2.;                                                                     % Tail height (ft)
XW = 4.;                                                                     % Distance from the wing to the radome tangency point (ft) 
% XCG = 10.;                                                                 % Distance from the nose to the missile center of gravity (ft)
XNCG = 1;                                                                    % Desired acceleration 
XHL = 19.5;                                                                  % Distance from the nose to the missile hinge line (ft)
% WGT = 1000.;                                                               % Missile mass (lb)

if ALT <=30000.
    RHO = 0.002378*exp(-ALT/30000.);                                         % Air density (slug/ft^3)
else
    RHO = 0.0034*exp(-ALT/22000.);    
end
%---------------

WACT = 150;
ZACT = 0.7;
TF = 3;
WCR = 50;
ZETA = 0.7;
TAU = 0.3;
XACC=XNCG;
%-----------
% DEL_Rad = 5/57.3;                                                            % Control surface deflection (radian)
SWING = 0.5*HW*(CTW+CRW);                                                    % Wing panel area
STAIL = 0.5*HT*(CTT+CRT);                                                    % Tail panel area
% SREF = pi*DIAM*DIAM/4.;                                                      % Reference area
SREF = m2ft*m2ft*0.13;                                                      % Reference area
XLP = FR*DIAM;                                                               % ?????
SPLAN = (XL-XLP)*DIAM + 1.33*XLP*DIAM/2.;                                    % Planform panel area
XCPN = 2*XLP/3;                                                              % Distance from the nose to the missile center of pressure for the nose
AN = 0.67*XLP*DIAM;                                                          % Nose area
AB = (XL-XLP)*DIAM;                                                          % Body area
XCPB = (0.67*AN*XLP+AB*(XLP+0.5*(XL-XLP)))/(AN+AB);                          % Distance from the nose to the missile center of pressure for the body
XCPW = XLP+XW+0.7*CRW-0.2*CTW;                                               % Distance from the nose to the missile center of pressure for the wing
% XMACH = V_M/A;                                                               % Mach number
% XIYY = WGT*(3*((DIAM/2)^2)+XL*XL)/(12*32.2);                                 % Missile moment of inertia
TMP1 = (XCG-XCPW)/DIAM;                                                      % Moment coefficient formula simlifier 1
TMP2 = (XCG-XHL)/DIAM;                                                       % Moment coefficient formula simlifier 2 
TMP3 = (XCG-XCPB)/DIAM;                                                      % Moment coefficient formula simlifier 3
TMP4 = (XCG-XCPN)/DIAM;                                                      % Moment coefficient formula simlifier 4
B = sqrt(XMACH^2-1);                                                         % Normalized speed 
Q = 0.5*RHO*V_M*V_M;                                                         % Dynamic pressure 
Y1 = 2 + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);                                % C_Nα
Y2 = 1.5*SPLAN/SREF;                                                         % C_Nα^2
Y3 = 8*STAIL/(B*SREF);                                                       % C_Nδ
Y4 = 2*TMP4 + 8*SWING*TMP1/(B*SREF) + 8*STAIL*TMP2/(B*SREF);                 % C_Mα
Y5 = 1.5*SPLAN*TMP3/SREF;                                                    % C_Mα^2
Y6 = 8*STAIL*TMP2/(B*SREF);                                                  % C_Mδ
P1 = WGT*XNCG/(Q*SREF)                                                        %
P2 = Y2-Y3*Y5/Y6;                                                            %
P3 = Y1-Y3*Y4/Y6;                                                            %
ALFTR = (-P3+sqrt(P3*P3+4.*P2*P1))/(2*P2);                                   % Trim angle of attack
DELTR = -Y4*ALFTR/Y6 - Y5*ALFTR*ALFTR/Y6;                                    %
% CNA = 2 + 1.5*SPLAN*ALFTR/SREF + 8*SWING/(B*SREF) + 8*STAIL/(B*SREF);        % C_N∝
% CND = 8*STAIL/(B*SREF);                                                      % C_Nδ
% CMA = 2*TMP4 + 1.5*SPLAN*ALFTR*TMP3/SREF + 8*SWING*TMP1/(B*SREF)...          % C_M∝
%       + 8*STAIL*TMP2/(B*SREF);                                               % ...
% CMD = 8*STAIL*TMP2/(B*SREF);                                                 % C_Mδ
XMA = Q*SREF*DIAM*CMA/XIYY;                                                  % M_∝
XMD = Q*SREF*DIAM*CMD/XIYY;                                                  % M_δ
ZA = -32.2*Q*SREF*CNA/(WGT*V_M);                                             % Z_∝ 
ZD = -32.2*Q*SREF*CND/(WGT*V_M);                                             % Z_δ
WZ = sqrt((XMA*ZD-ZA*XMD)/ZD);                                               % ω_z:  Airframe Zero 
WAF = sqrt(-XMA);                                                            % ω_AF: Airframe Natural Frequency
ZAF = 0.5*WAF*ZA/XMA;                                                        % ξ_AF: Airframe Damping
XK1 = -V_M*(XMA*ZD-XMD*ZA)/(1845*XMA);                                       % Gain K1
XK2 = XK1;                                                                   % Gain K2
TA = XMD/(XMA*ZD-XMD*ZA);                                                    % T_∝
XK3 = 1845*XK1/V_M;                                                          % Gain K3
W=(TAU*WCR*(1+2.*ZAF*WAF/WCR)-1)/(2*ZETA*TAU);                               %
W0=W/sqrt(TAU*WCR);                                                          %
Z0=.5*W0*(2*ZETA/W+TAU-WAF^2/(W0*W0*WCR));                                   %
XKC=(-W0^2/WZ^2-1.+2.*Z0*W0*TA)/(1.-2.*Z0*W0*TA+W0*W0*TA*TA);                %
XKA=XK3/(XK1*XKC);                                                           %
XK0=-W*W/(TAU*WAF*WAF);                                                      %
XK=XK0/(XK1*(1+XKC));                                                        %
WI=XKC*TA*W0*W0/(1+XKC+W0^2/WZ^2);                                           %
XKR=XK/(XKA*WI);                                                             %
XKDC=1.+1845./(XKA*V_M);                                                     %

% % Run Simulink
% simOut = sim('airframe');
% % Store Simulink data in arrays
% SimArrayT = simOut.tout;
% SimArrayXNL = simOut.ScopeData_MissileAcc.signals.values;
% SimArrayTHD = simOut.ScopeData_ThetaDot.signals.values;

E = 0.;                                                                      % e from chain rule 
ED = 0.;                                                                     % e_dot
DELD = 0;                                                                    %
DEL = 0;                                                                     %
T = 0;                                                                       % Time
H = .00001;                                                                  % Time step  
X = 0;                                                                       %
while T <= (TF-.00001)
    E_OLD = E;
    ED_OLD = ED;
    DEL_OLD = DEL;
    DELD_OLD = DELD;
    X_OLD = X;

    THD = XK3*(E+TA*ED);                                                     % Derivative of the theta angle
    DELC = XKR*(X+THD);                                                      %
    DELDD = WACT*WACT*(DELC-DEL-2.*ZACT*DELD/WACT);                          %
    EDD = WAF*WAF*(DEL-E-2.*ZAF*ED/WAF);                                     %
    XNL = XK1*(E-(EDD/WZ^2));                                             % Missile acceleration
    XD = WI*(THD+XKA*(XNL-XNCG*XKDC));                                        %

%---------------------------------------------------------------------------
% Save T, XNL, THD in an array   
    n = n+1;
    ArrayT(n) = T;
    ArrayXNL(n) = XNL;
    ArrayXCG(n) = XNCG;
%---------------------------------------------------------------------------
% Integrate E and ED using "Euler Method"
    coeff = 1.0;   
    E = coeff*(E_OLD + H*ED);
    ED = coeff*(ED_OLD + H*EDD);
    DEL = coeff*(DEL_OLD+H*DELD);
    DELD = coeff*(DELD_OLD+H*DELDD);
    X = coeff*(X_OLD+H*XD);
    T = T+H;
%---------------------------------------------------------------------------
end

figure
plot(ArrayT,ArrayXNL,ArrayT,ArrayXCG),grid
xlabel('Time (Sec)')
ylabel('Acceleration (G) ')
clc
output=[ArrayT',ArrayXNL',ArrayXCG'];
disp 'simulation finished'




