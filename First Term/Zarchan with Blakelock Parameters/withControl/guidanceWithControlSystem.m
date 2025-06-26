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

ALT = ALT_array(user_index);
V_M_b = V_M_array(user_index);
XMACH = XMACH_array(user_index);
WGT = WGT_array(user_index);
XIYY = XIYY_array(user_index);
CMA = CMA_array(user_index);
CNA = CNA_array(user_index);
CMD = CMD_array(user_index);
CND = CND_array(user_index);
XCG = XCG_array(user_index);

%==================================================================================
% Standard Atmosphere, SI Units Table Values from Anderson's Fundamentals of Aerodynamics

ALT_table = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, ...
        1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, ...
        2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, ...
        3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900, ...
        4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900, ...
        5000, 5100, 5200, 5400, 5500, 5600, 5700, 5800, 5900, 6000, ...
        6100, 6200, 6300, 6400, 6500, 6600, 6700, 6800, 6900, 7000, ...
        7100, 7200, 7300, 7400, 7500, 7600, 7700, 7800, 7900, 8000, ...
        8100, 8200, 8300, 8400, 8500, 8600, 8700, 8800, 8900, 9000, ...
        9100, 9200, 9300, 9400, 9500, 9600, 9700, 9800, 9900, 10000, ...
        10100, 10200, 10300, 10400, 10500, 10600, 10700, 10800, 10900, 11000, ...
        11100, 11200, 11300, 11400, 11500, 11600, 11700, 11800, 11900, 12000, ...
        12100, 12200, 12300, 12400, 12500, 12600, 12700, 12800, 12900, 13000, ...
        13100, 13200, 13300, 13400, 13500, 13600, 13700, 13800, 13900, 14000, ...
        14100, 14200, 14300, 14400, 14500, 14600, 14700, 14800, 14900, 15000, ...
        15100, 15200, 15300, 15400, 15500, 15600, 15700, 15800, 15900, 16000, ...
        16100, 16200, 16300, 16400, 16500, 16600, 16700, 16800, 16900, 17000, ...
        17100, 17200, 17300, 17400, 17500, 17600, 17700, 17800, 17900, 18000, ...
        18100, 18200, 18300, 18400, 18500, 18600, 18700, 18800, 18900, 19000, ...
        19100, 19200, 19300, 19400, 19500, 19600, 19700, 19800, 19900, 20000, ...
        20200, 20400, 20600, 20800, 21000, 21200, 21400, 21600, 21800, 22000, ...
        22200, 22400, 22600, 22800, 23000, 23200, 23400, 23600, 23800, 24000, ...
        24200, 24400, 24600, 24800, 25000, 25200, 25400, 25600, 25800, 26000, ...
        26200, 26400, 26600, 26800, 27000, 27200, 27400, 27600, 27800, 28000, ...
        28200, 28400, 28600, 28800, 29000, 29200, 29400, 29600, 29800, 30000, ...
        30200, 30400, 30600, 30800, 31000, 31200, 31400, 31600, 31800, 32000, ...
        32200, 32400, 32600, 32800, 33000, 33200, 33400, 33600, 33800, 34000, ...
        34200, 34400, 34600, 34800, 35000, 35200, 35400, 35600, 35800, 36000, ...
        36200, 36400, 36600, 36800, 37000, 37200, 37400, 37600, 37800, 38000, ...
        38200, 38400, 38600, 38800, 39000, 39200, 39400, 39600, 39800, 40000, ...
        40200, 40400, 40600, 40800, 41000, 41200, 41400, 41600, 41800, 42000, ...
        42200, 42400, 42600, 42800, 43000, 43200, 43400, 43600, 43800, 44000, ...
        44200, 44400, 44600, 44800, 45000, 45200, 45400, 45600, 45800, 46000, ...
        46200, 46400, 46600, 46800, 47000, 47200, 47400, 47600, 47800, 48000, ...
        48200, 48400, 48600, 48800, 49000, 49200, 49400, 49600, 49800, 50000, ...
        50500, 51000, 51500, 52000, 52500, 53000, 53500, 54000, 54500, 55000, ...
        55500, 56000, 56500, 57000, 57500, 58000, 58500, 59000, 59500];


RHO_table = [1.2250 * 10^0, 1.2133 * 10^0, 1.2071 * 10^0, 1.1901 * 10^0, ...
        1.1787 * 10^0, 1.1673 * 10^0, 1.1560 * 10^0, 1.1448 * 10^0, ...
        1.1337 * 10^0, 1.1226 * 10^0, 1.1117 * 10^0, 1.1008 * 10^0, ...
        1.0900 * 10^0, 1.0793 * 10^0, 1.0687 * 10^0, 1.0581 * 10^0, ...
        1.0476 * 10^0, 1.0373 * 10^0, 1.0269 * 10^0, 1.0167 * 10^0, ...
        1.0066 * 10^0, 9.9649 * 10^-1, 9.8649 * 10^-1, 9.7657 * 10^-1, ...
        9.6673 * 10^-1, 9.5696 * 10^-1, 9.4727 * 10^-1, 9.3765 * 10^-1, ...
        9.2811 * 10^-1, 9.1865 * 10^-1, 9.0926 * 10^-1, 8.9994 * 10^-1, ...
        8.9070 * 10^-1, 8.8153 * 10^-1, 8.7243 * 10^-1, 8.6341 * 10^-1, ...
        8.5445 * 10^-1, 8.4557 * 10^-1, 8.3676 * 10^-1, 8.2802 * 10^-1, ...
        8.1935 * 10^-1, 8.1075 * 10^-1, 8.0222 * 10^-1, 7.9376 * 10^-1, ...
        7.8536 * 10^-1, 7.7704 * 10^-1, 7.6878 * 10^-1, 7.6059 * 10^-1, ...
        7.5247 * 10^-1, 7.4442 * 10^-1, 7.3643 * 10^-1, 7.2851 * 10^-1, ...
        7.2065 * 10^-1, 7.0513 * 10^-1, 6.9747 * 10^-1, 6.8987 * 10^-1, ...
        6.8234 * 10^-1, 6.7486 * 10^-1, 6.6746 * 10^-1, 6.6011 * 10^-1, ...
        6.5283 * 10^-1, 6.4561 * 10^-1, 6.3845 * 10^-1, 6.3135 * 10^-1, ...
        6.2431 * 10^-1, 6.1733 * 10^-1, 6.1041 * 10^-1, 6.0356 * 10^-1, ...
        5.9676 * 10^-1, 5.9002 * 10^-1, 5.8334 * 10^-1, 5.7671 * 10^-1, ...
        5.7015 * 10^-1, 5.6364 * 10^-1, 5.5719 * 10^-1, 5.5080 * 10^-1, ...
        5.4446 * 10^-1, 5.3818 * 10^-1, 5.3195 * 10^-1, 5.2578 * 10^-1, ...
        5.1967 * 10^-1, 5.1361 * 10^-1, 5.0760 * 10^-1, 5.0165 * 10^-1, ...
        4.9575 * 10^-1, 4.8991 * 10^-1, 4.8412 * 10^-1, 4.7838 * 10^-1, ...
        4.7269 * 10^-1, 4.6706 * 10^-1, 4.6148 * 10^-1, 4.5595 * 10^-1, ...
        4.5047 * 10^-1, 4.4504 * 10^-1, 4.3966 * 10^-1, 4.3433 * 10^-1, ...
        4.2905 * 10^-1, 4.2382 * 10^-1, 4.1864 * 10^-1, 4.1351 * 10^-1, ...
        4.0842 * 10^-1, 4.0339 * 10^-1, 3.9840 * 10^-1, 3.9346 * 10^-1, ...
        3.8857 * 10^-1, 3.8372 * 10^-1, 3.7892 * 10^-1, 3.7417 * 10^-1, ...
        3.6946 * 10^-1, 3.6480 * 10^-1, 3.5932 * 10^-1, 3.5371 * 10^-1, ...
        3.4820 * 10^-1, 3.4277 * 10^-1, 3.3743 * 10^-1, 3.3217 * 10^-1, ...
        3.2699 * 10^-1, 3.2189 * 10^-1, 3.1687 * 10^-1, 3.1194 * 10^-1, ...
        3.0707 * 10^-1, 3.0229 * 10^-1, 2.9758 * 10^-1, 2.9294 * 10^-1, ...
        2.8837 * 10^-1, 2.8388 * 10^-1, 2.7945 * 10^-1, 2.7510 * 10^-1, ...
        2.7081 * 10^-1, 2.6659 * 10^-1, 2.6244 * 10^-1, 2.5835 * 10^-1, ...
        2.5433 * 10^-1, 2.5036 * 10^-1, 2.4646 * 10^-1, 2.4262 * 10^-1, ...
        2.3884 * 10^-1, 2.3512 * 10^-1, 2.3146 * 10^-1, 2.2785 * 10^-1, ...
        2.2430 * 10^-1, 2.2081 * 10^-1, 2.1737 * 10^-1, 2.1399 * 10^-1, ...
        2.1065 * 10^-1, 2.0737 * 10^-1, 2.0414 * 10^-1, 2.0096 * 10^-1, ...
        1.9783 * 10^-1, 1.9475 * 10^-1, 1.9172 * 10^-1, 1.8874 * 10^-1, ...
        1.8580 * 10^-1, 1.8290 * 10^-1, 1.8006 * 10^-1, 1.7725 * 10^-1, ...
        1.7449 * 10^-1, 1.7178 * 10^-1, 1.6910 * 10^-1, 1.6647 * 10^-1, ...
        1.6388 * 10^-1, 1.6133 * 10^-1, 1.5882 * 10^-1, 1.5634 * 10^-1, ...
        1.5391 * 10^-1, 1.5151 * 10^-1, 1.4916 * 10^-1, 1.4683 * 10^-1, ...
        1.4455 * 10^-1, 1.4230 * 10^-1, 1.4009 * 10^-1, 1.3791 * 10^-1, ...
        1.3576 * 10^-1, 1.3365 * 10^-1, 1.3157 * 10^-1, 1.2952 * 10^-1, ...
        1.2751 * 10^-1, 1.2552 * 10^-1, 1.2357 * 10^-1, 1.2165 * 10^-1, ...
        1.1975 * 10^-1, 1.1789 * 10^-1, 1.1606 * 10^-1, 1.1425 * 10^-1, ...
        1.1247 * 10^-1, 1.1072 * 10^-1, 1.0900 * 10^-1, 1.0731 * 10^-1, ...
        1.0564 * 10^-1, 1.0399 * 10^-1, 1.0238 * 10^-1, 1.0079 * 10^-1, ...
        9.9218 * 10^-2, 9.7675 * 10^-2, 9.6156 * 10^-2, 9.4661 * 10^-2, ...
        9.3189 * 10^-2, 9.1740 * 10^-2, 9.0313 * 10^-2, 8.8909 * 10^-2, ...
        8.6166 * 10^-2, 8.3508 * 10^-2, 8.0931 * 10^-2, 7.8435 * 10^-2, ...
        7.6015 * 10^-2, 7.3671 * 10^-2, 7.1399 * 10^-2, 6.9197 * 10^-2, ...
        6.7063 * 10^-2, 6.4995 * 10^-2, 6.2991 * 10^-2, 6.1049 * 10^-2, ...
        5.9167 * 10^-2, 5.7343 * 10^-2, 5.5575 * 10^-2, 5.3862 * 10^-2, ...
        5.2202 * 10^-2, 5.0593 * 10^-2, 4.9034 * 10^-2, 4.7522 * 10^-2, ...
        4.6058 * 10^-2, 4.4639 * 10^-2, 4.3263 * 10^-2, 4.1931 * 10^-2, ...
        4.0639 * 10^-2, 3.9333 * 10^-2, 3.8020 * 10^-2, 3.6755 * 10^-2, ...
        3.5535 * 10^-2, 3.4359 * 10^-2, 3.3225 * 10^-2, 3.2131 * 10^-2, ...
        3.1076 * 10^-2, 3.0059 * 10^-2, 2.9077 * 10^-2, 2.8130 * 10^-2, ...
        2.7217 * 10^-2, 2.6335 * 10^-2, 2.5484 * 10^-2, 2.4663 * 10^-2, ...
        2.3871 * 10^-2, 2.3106 * 10^-2, 2.2367 * 10^-2, 2.1654 * 10^-2, ...
        2.0966 * 10^-2, 2.0301 * 10^-2, 1.9659 * 10^-2, 1.9039 * 10^-2, ...
        1.8440 * 10^-2, 1.7861 * 10^-2, 1.7302 * 10^-2, 1.6762 * 10^-2, ...
        1.6240 * 10^-2, 1.5734 * 10^-2, 1.5278 * 10^-2, 1.4777 * 10^-2, ...
        1.4321 * 10^-2, 1.3881 * 10^-2, 1.3455 * 10^-2, 1.3044 * 10^-2, ...
        1.2646 * 10^-2, 1.2261 * 10^-2, 1.1889 * 10^-2, 1.1529 * 10^-2, ...
        1.1180 * 10^-2, 1.0844 * 10^-2, 1.0518 * 10^-2, 1.0202 * 10^-2, ...
        9.8972 * 10^-3, 9.6020 * 10^-3, 9.3162 * 10^-3, 9.0396 * 10^-3, ...
        8.7720 * 10^-3, 8.5128 * 10^-3, 8.2620 * 10^-3, 8.0191 * 10^-3, ...
        7.7839 * 10^-3, 7.5562 * 10^-3, 7.3357 * 10^-3, 7.1221 * 10^-3, ...
        6.9152 * 10^-3, 6.7149 * 10^-3, 6.5208 * 10^-3, 6.3328 * 10^-3, ...
        6.1506 * 10^-3, 5.9741 * 10^-3, 5.8030 * 10^-3, 5.6373 * 10^-3, ...
        5.4767 * 10^-3, 5.3210 * 10^-3, 5.1701 * 10^-3, 5.0238 * 10^-3, ...
        4.8820 * 10^-3, 4.7445 * 10^-3, 4.6112 * 10^-3, 4.4819 * 10^-3, ...
        4.3566 * 10^-3, 4.2350 * 10^-3, 4.1171 * 10^-3, 4.0028 * 10^-3, ...
        3.8919 * 10^-3, 3.7843 * 10^-3, 3.6799 * 10^-3, 3.5786 * 10^-3, ...
        3.4804 * 10^-3, 3.3850 * 10^-3, 3.2925 * 10^-3, 3.2027 * 10^-3, ...
        3.1156 * 10^-3, 3.0310 * 10^-3, 2.9489 * 10^-3, 2.8692 * 10^-3, ...
        2.7918 * 10^-3, 2.7167 * 10^-3, 2.6438 * 10^-3, 2.5730 * 10^-3, ...
        2.5042 * 10^-3, 2.4374 * 10^-3, 2.3726 * 10^-3, 2.3096 * 10^-3, ...
        2.2484 * 10^-3, 2.1889 * 10^-3, 2.1312 * 10^-3, 2.0751 * 10^-3, ...
        2.0206 * 10^-3, 1.9677 * 10^-3, 1.9162 * 10^-3, 1.8662 * 10^-3, ...
        1.8177 * 10^-3, 1.7704 * 10^-3, 1.7246 * 10^-3, 1.6799 * 10^-3, ...
        1.6366 * 10^-3, 1.5944 * 10^-3, 1.5535 * 10^-3, 1.5136 * 10^-3, ...
        1.4757 * 10^-3, 1.4409 * 10^-3, 1.4070 * 10^-3, 1.3739 * 10^-3, ...
        1.3416 * 10^-3, 1.3100 * 10^-3, 1.2792 * 10^-3, 1.2491 * 10^-3, ...
        1.2197 * 10^-3, 1.1910 * 10^-3, 1.1630 * 10^-3, 1.1357 * 10^-3, ...
        1.1089 * 10^-3, 1.0829 * 10^-3, 1.0203 * 10^-3, 9.6140 * 10^-4, ...
        9.0589 * 10^-4, 8.5360 * 10^-4, 8.0433 * 10^-4, 7.5791 * 10^-4, ...
        7.1478 * 10^-4, 6.7867 * 10^-4, 6.4412 * 10^-4, 6.1108 * 10^-4, ...
        5.7949 * 10^-4, 5.4931 * 10^-4, 5.2047 * 10^-4, 4.9293 * 10^-4, ...
        4.6664 * 10^-4, 4.4156 * 10^-4, 4.1763 * 10^-4, 3.9482 * 10^-4, ...
        3.7307 * 10^-4];
%===========================================================================


%============================================================================================= 
% User Inputs:
R_Mx = ft2m*0.;                           % Missile Position X-Component (m)
R_My = ALT;                               % Missile Position Y-Component (m) 
R_Tx = ft2m*40000.;                       % Target Position X-Component (m)
R_Ty = ft2m*10000.;                       % Target Position Y-Component (m)
V_M = V_M_b;                              % Velocity of the missile (m/s)
V_T = ft2m*1000.;                         % Velocity of the target (m/s)
Beta = -20.;                              % Angle of the target velocity (degree)
XNT = -5;                                 % Target maneuver
HE = 40.;                                % Heading error (degree)
XNP = 3.;                                 % Effective navigation ratio or gain (Range of 3-5)

%============================================================================================= 
TF = 1;                                   % Final simulation time (s)

% Control Systems Parameters
XNCG = 1;                                 % Desired acceleration (m/s^2)
WACT = 150;                               % Actuator natural frequency (rad/s)
ZACT = 0.7;                               % Actuator damping
WCR = 50;                                 % Crossover Frequency (rad/s)
ZETA = 0.7;                               % Actuator damping
TAU = 0.3;                                % Time constant (s)

%============================================================================================= 
% Calculations

DIAM = 0.203;                             % Missile diameter (m)
SREF = 0.13;                              % Reference area (m^2)
RHO = interp1(ALT_table,RHO_table, ALT);  % Density (kg/m^3)
Q = 0.5*RHO*V_M*V_M;                      % Dynamic pressure 

XMA = Q*SREF*DIAM*CMA/XIYY;               % M_∝ (1/s^2)
XMD = Q*SREF*DIAM*CMD/XIYY;               % M_δ (1/s^2)
ZA = -Q*SREF*CNA/((WGT)*V_M);             % Z_∝ (1/s)
ZD = -Q*SREF*CND/((WGT)*V_M);             % Z_δ (1/s)
WZ = sqrt((XMA*ZD-ZA*XMD)/ZD);            % ω_z:  Airframe Zero (rad/s)
WAF = sqrt(-XMA);                         % ω_AF: Airframe Natural Frequency (rad/s)
ZAF = 0.5*WAF*ZA/XMA;                     % ξ_AF: Airframe Damping
XK1 = -V_M*(XMA*ZD-XMD*ZA)/((g*r2d)*XMA); % Gain K1: Aerodynamic acceleration gain (g/deg)
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
XKDC = 1 + g*r2d / (XKA*V_M);             % Steady-state gain for autopilot


% XKR -> 1/s
% WI -> rad/s
% XKA -> dimensionless
% XKDC ->dimensionless

%============================================================================================= 
% Create Main folder to store simulation results
mainFolder = 'guidanceWithControlSystem_simulationResults';
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
    sprintf('TF = %.9f (Final simulation time, s)', TF);
    sprintf('XNCG = %.9f (Desired acceleration, m/s^2)', XNCG);
    sprintf('WACT = %.9f (Actuator natural frequency, rad/s)', WACT);
    sprintf('ZACT = %.9f (Actuator damping)', ZACT);
    sprintf('WCR = %.9f (Crossover frequency, rad/s)', WCR);
    sprintf('ZETA = %.9f (Actuator damping)', ZETA);
    sprintf('TAU = %.9f (Time constant, s)', TAU);
    sprintf('XMACH = %.9f (Mach number)', XMACH);
    sprintf('WGT = %.9f (Weight of the missile, kg)', WGT);
    sprintf('XIYY = %.9f (Moment of inertia, kg·m^2)', XIYY);
    sprintf('CMA = %.9f (Pitch moment derivative with respect to angle of attack, 1/rad)', CMA);
    sprintf('CNA = %.9f (Normal force derivative with respect to angle of attack, 1/rad)', CNA);
    sprintf('CMD = %.9f (Pitch moment derivative with respect to deflection angle, 1/rad)', CMD);
    sprintf('CND = %.9f (Normal force derivative with respect to deflection angle, 1/rad)', CND);
    sprintf('XCG = %.9f (Center of gravity position, m)', XCG);
    sprintf('RHO = %.9f (Air density, kg/m^3)', RHO);    
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
simOut = sim('proportionalNav_withControlSystem');

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
SimArrayXNLG = simOut.ScopeData_XNLG.signals.values;
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

n = 0;                                    % Array number initialize
E = 0.;                                   % e from chain rule 
ED = 0.;                                  % e_dot
DELD = 0;                                 % Control surface deflection derivative (degree)
DEL = 0;                                  % Control surface deflection (degree)
T = 0;                                    % Initial time (s)
X = 0;                                    % Initial missile state

% Integration using "Runge Kutta 4th Order Method"     
while V_C >= 0. 
    if R_TM < 2500*ft2m 
        H = 0.001;                        % Time step (s)  
    else
        H = 0.001;
    end

    X_s = [BetaRad, R_Tx, R_Ty, R_Mx, R_My, V_Mx, V_My, E, ED, DEL, DELD, X];
    X_b = X_s;
    L = length(X_b);

    BetaRad = X_s(1);
    R_Tx = X_s(2);
    R_Ty = X_s(3);
    R_Mx = X_s(4);
    R_My = X_s(5);
    V_Mx = X_s(6);
    V_My = X_s(7);
    E = X_s(8);
    ED = X_s(9);
    DEL = X_s(10);
    DELD = X_s(11);
    X = X_s(12); 

    T = T + H;                            % Increase time with step size (s)  
     
    R_TMx = R_Tx - R_Mx;                  % Length of the Line of sight X-Component (m) 
    R_TMy = R_Ty - R_My;                  % Length of the Line of sight Y-Component (m) 
    R_TM = sqrt(R_TMx*R_TMx + R_TMy*R_TMy);           % Length of the Line of sight (m) 
    V_TMx = V_Tx - V_Mx;                  % Velocity of the Line of sight X-Component (m/s) 
    V_TMy = V_Ty - V_My;                  % Velocity of the Line of sight Y-Component (m/s) 
    V_C = -(R_TMx*V_TMx + R_TMy*V_TMy)/R_TM;                       % Closing velocity (m/s) 
    Lamda = atan2(R_TMy,R_TMx);           % Line-of-sight angle (rad) 
    Lamda_d = (V_TMy*R_TMx - V_TMx*R_TMy)/(R_TM*R_TM); % Time derivative of lamda angle
    XNCD = XNP*V_C*Lamda_d;               % Desired acceleration command (m/s^2)
    XNCG = XNCD / g;                      % Desired acceleration command (g)
    
    THD = XK3*(E+TA*ED);                  % Derivative of the theta angle
    DELC = XKR*(X+THD);                                                  
    DELDD = WACT*WACT*(DELC-DEL-2.*ZACT*DELD/WACT);                      
    EDD = WAF*WAF*(DEL-E-2.*ZAF*ED/WAF);  
    XNL = XK1*(E-(EDD/WZ^2));             % Missile acceleration (g)
    XD = WI*(THD+XKA*(XNL-XNCG*XKDC));                                   
        
    XNC = XNL * g;                        % Missile acceleration (m/s^2)
    A_Mx = -XNC*sin(Lamda);               % Missile acceleration X-Component (m/s^2)
    A_My =  XNC*cos(Lamda);               % Missile acceleration Y-Component (m/s^2)
    V_Tx = -V_T*cos(BetaRad);             % Target velocity X-Component (m/s) 
    V_Ty =  V_T*sin(BetaRad);             % Target velocity Y-Component (m/s) 
    Beta_d = XNT/V_T;                     % Time derivative of the Beta angle 


    % Calculate K1
    K1 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My, ED, EDD, DELD, DELDD, XD];
    
    % Calculate K2
    for i = 1:L
        X_t(i) = X_b(i) + 0.5 * H * K1(i);
    end
    K2 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My, ED, EDD, DELD, DELDD, XD];
    
    % Calculate K3
    for i = 1:L
        X_t(i) = X_b(i) + 0.5 * H * K2(i);
    end
    K3 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My, ED, EDD, DELD, DELDD, XD];
    
    % Calculate K4
    for i = 1:L
        X_t(i) = X_b(i) + H * K3(i);
    end
    K4 = [Beta_d, V_Tx, V_Ty, V_Mx, V_My, A_Mx, A_My, ED, EDD, DELD, DELDD, XD];

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
    E = X_s(8);
    ED = X_s(9);
    DEL = X_s(10);
    DELD = X_s(11);
    X = X_s(12); 

    n = n + 1; 
    
    % Store MATLAB data in arrays
    ArrayT(n) = T;                        % Store Time
    ArrayBeta_Rad(n) = BetaRad;           % Store Beta Angle
    ArrayLamda(n) = Lamda;                % Store Lamda
    ArrayLamda_D(n) = Lamda_d;            % Store Lamda_d    
    ArrayR_Tx(n) = R_Tx;                  % Store R_Tx
    ArrayR_Ty(n) = R_Ty;                  % Store R_Ty 
    ArrayR_Mx(n) = R_Mx;                  % Store R_Mx 
    ArrayR_My(n) = R_My;                  % Store R_My 
    ArrayR_TM(n) = R_TM;                  % Store R_TM
    ArrayV_Tx(n) = V_Tx;                  % Store V_Tx
    ArrayV_Ty(n) = V_Ty;                  % Store V_Ty
    ArrayV_Mx(n) = V_Mx;                  % Store V_Mx
    ArrayV_My(n) = V_My;                  % Store V_My
    ArrayV_C(n) = V_C;                    % Store V_C
    ArrayXNLG(n) = XNL;                   % Store XNLG
    ArrayA_Mx(n) = A_Mx;                  % Store A_Mx  
    ArrayA_My(n) = A_My;                  % Store A_My  

end 

%============================================================================================= 
% Write all results in a text files 

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

%================================================================================== 
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
%================================================================================== 
% PLOT 2: Target and Missile Acceleration (MATLAB vs Simulink)

figure;
hold on;

% Plot Simulink XNCG
plot(SimArrayT, SimArrayXNLG, 'g-', 'DisplayName', 'Simulink Missile Acceleration');      

% Plot MATLAB XNCG
plot(ArrayT, ArrayXNLG, 'r--', 'DisplayName', 'MATLAB Missile Acceleration ');        

grid on;
title('Missile Acceleration Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('Acceleration of missile (G)');
legend;
hold off;
%================================================================================== 
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
%================================================================================== 
% PLOT 4: Closing Velocity (MATLAB vs Simulink)

figure;
hold on;

% Plot Simulink V_C
plot(SimArrayT, SimArrayV_C, 'g-', 'DisplayName', 'Simulink V_C');      

% Plot MATLAB V_C
plot(ArrayT, ArrayV_C, 'r--', 'DisplayName', 'MATLAB V_C');           

grid on;
title('V_C Over Time: MATLAB vs. Simulink');
xlabel('Time (s)');
ylabel('V_C (ft/s)');
legend;
hold off;
%================================================================================== 
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
%================================================================================== 
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
%================================================================================== 

% %--------------------------------------------------------------------------  
% %-------------------------------------------------------------------------- PLOT 7 
% 
% figure;
% hold on;
% 
% % Plot Simulink V_Mx
% plot(SimArrayT, SimArrayV_Mx, 'g-', 'DisplayName', 'Simulink V_Mx');        % Green line for Simulink V_Mx
% 
% % Plot MATLAB V_Mx
% plot(ArrayT, ArrayV_Mx, 'r--', 'DisplayName', 'MATLAB V_Mx');              % Red dashed line for MATLAB V_Mx
% 
% grid on;
% title('V_Mx Over Time: MATLAB vs. Simulink');
% xlabel('Time (s)');
% ylabel('V_Mx (ft/s)');
% legend;
% hold off;
% 
% %--------------------------------------------------------------------------  
% %-------------------------------------------------------------------------- PLOT 8 
% 
% figure;
% hold on;
% 
% % Plot Simulink V_My
% plot(SimArrayT, SimArrayV_My, 'g-', 'DisplayName', 'Simulink V_My');        % Green line for Simulink V_My
% 
% % Plot MATLAB V_My
% plot(ArrayT, ArrayV_My, 'r--', 'DisplayName', 'MATLAB V_My');              % Red dashed line for MATLAB V_My
% 
% grid on;
% title('V_My Over Time: MATLAB vs. Simulink');
% xlabel('Time (s)');
% ylabel('V_My (ft/s)');
% legend;
% hold off;
% 
% %-------------------------------------------------------------------------- PLOT 9 
% 
% figure;
% hold on;
% 
% % Plot Simulink Target Path
% plot(SimArrayV_Tx, SimArrayV_Ty, 'g-', 'DisplayName', 'Simulink Target Velocity');          % Green line for Simulink target path
% % Plot Simulink Missile Path
% plot(SimArrayV_Mx, SimArrayV_My, 'b-', 'DisplayName', 'Simulink Missile Velocity');         % Blue line for Simulink missile path
% 
% % Plot MATLAB Target Path
% plot(ArrayV_Tx, ArrayV_Ty, 'm--', 'DisplayName', 'MATLAB Target Velocity ');                 % Magenta dashed line for MATLAB target path
% % Plot MATLAB Missile Path
% plot(ArrayV_Mx, ArrayV_My, 'k--', 'DisplayName', 'MATLAB Missile Velocity ');                % Black dashed line for MATLAB missile path
% 
% grid on;
% title('Missile-Target Velocity Simulation: MATLAB vs. Simulink');
% xlabel('Downrange (Ft)');
% ylabel('Altitude (Ft)');
% legend;
% hold off;
% 
% %-------------------------------------------------------------------------- PLOT 10 
% 
% figure;
% hold on;
% 
% % Plot Simulink Lamda_D
% plot(SimArrayT, SimArrayLamda_D, 'g-', 'DisplayName', 'Simulink Lamda_D');          % Green line for Simulink Lamda_D
% 
% % Plot MATLAB Lamda_D
% plot(ArrayT, ArrayLamda_D, 'm--', 'DisplayName', 'MATLAB Lamda_D ');                 % Magenta dashed line for MATLAB Lamda_D
% 
% grid on;
% title('Lamda_D: MATLAB vs. Simulink');
% xlabel('Downrange (Ft)');
% ylabel('Altitude (Ft)');
% legend;
% hold off;
% 
% %-------------------------------------------------------------------------- PLOT 11 
% 
% figure;
% hold on;
% 
% % Plot Simulink Lamda
% plot(SimArrayT, SimArrayLamda, 'g-', 'DisplayName', 'Simulink Lamda');          % Green line for Simulink Lamda
% 
% % Plot MATLAB Lamda
% plot(ArrayT, ArrayLamda, 'm--', 'DisplayName', 'MATLAB Lamda ');                 % Magenta dashed line for MATLAB Lamda
% 
% grid on;
% title('Lamda: MATLAB vs. Simulink');
% xlabel('Downrange (Ft)');
% ylabel('Altitude (Ft)');
% legend;
% hold off;
