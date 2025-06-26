clear; clc; close all;
diameter_L = 0.165;                             % m
Sref = pi*diameter_L^2/4
nose_L   = 0.50;                                % m
nose_rho = 2768;                                % kg/m^3
nose_xcg = (nose_L/3)*2                         % m

GNC_L   = 0.50;                                 % m
GNC_rho = 2768;                                 % kg/m^3
GNC_xcg = GNC_L/2 + nose_xcg                    % m

warhead_L   = 0.80;                             % m
warhead_rho = 3600;                             % kg/m^3
warhead_xcg = warhead_L/2 + GNC_xcg             % m

empty_full_L   = 1.20;                          % m
empty_fuel_rho = 830;                           % kg/m^3
empty_fuel_xcg = empty_full_L/2 + warhead_xcg   % m

full_fuel_L   = 1.20;                           % m
full_fuel_rho = 2214;                           % kg/m^3 
full_fuel_xcg = full_fuel_L/2 + warhead_xcg     % m

% ===== Fin =====
rho_aluminum = 1937;                            % kg/m^3
fin_thickness = 0.005;                          % 5 mm

% ===== Front Fins Geometry =====
% Rectangular base
rect_x = 0.20;                          % Start x-location
rect_width = 0.20;                     % Along body axis
rect_span = 0.05;                      % Perpendicular (span)

% Triangular extension
tri_base = 0.70;                       % Base along body axis
tri_span = 0.20;                       % Span (height)

% === Areas ===
area_rect = rect_width * rect_span;
area_tri  = 0.5 * tri_base * tri_span;

% === Volumes ===
vol_rect = area_rect * fin_thickness;
vol_tri  = area_tri * fin_thickness;

% === Masses (single fin) ===
mass_rect = vol_rect * rho_aluminum;
mass_tri  = vol_tri * rho_aluminum;

% === CGs (from tip) ===
xcg_rect = rect_x + rect_width / 2;
xcg_tri  = rect_x + rect_width + tri_base / 3;

% === Combined fin CG (single fin) ===
mass_total = mass_rect + mass_tri;
xcg_single = (mass_rect * xcg_rect + mass_tri * xcg_tri) / mass_total;

% === Total mass for 4 fins ===
front_mass = mass_total * 4;
front_xcg = xcg_single;

% === Iyy (perpendicular to missile axis), single fin ===
Iyy_rect = (1/12) * mass_rect * rect_span^2;
Iyy_tri  = (1/36) * mass_tri * tri_span^2;

% === Parallel axis shift for each part (to missile CG)
d_rect = xcg_single - xcg_rect;
d_tri  = xcg_single - xcg_tri;

Iyy_total_single = Iyy_rect + mass_rect * d_rect^2 + ...
                   Iyy_tri + mass_tri * d_tri^2;

% === Total Iyy for 4 fins ===
Iyy_front_total = 4 * Iyy_total_single;

% === Rear Fins ===
rear_chord = 0.30;                              % m
rear_span  = 0.25;                              % m
rear_root  = 2.70;                              % m
rear_area = 0.5 * rear_chord * rear_span;
rear_volume = rear_area * fin_thickness;
rear_mass = rear_volume * rho_aluminum * 4;
rear_xcg = rear_root + rear_chord / 3;

nose_volume = 1/3 * pi * (diameter_L/2)^2 * nose_L;
nose_mass = nose_volume * nose_rho;

GNC_volume = pi * (diameter_L/2)^2 * GNC_L;
GNC_mass = GNC_volume * GNC_rho;

warhead_volume = pi * (diameter_L/2)^2 * warhead_L;
warhead_mass = warhead_volume * warhead_rho;

full_fuel_volume = pi * (diameter_L/2)^2 * full_fuel_L;
full_fuel_mass = full_fuel_volume * full_fuel_rho;

empty_fuel_volume = pi * (diameter_L/2)^2 * empty_full_L;
empty_fuel_mass = empty_fuel_volume * empty_fuel_rho

fuel_mass = full_fuel_mass - empty_fuel_mass

missile_mass_full = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + full_fuel_mass 
missile_mass_empty = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + empty_fuel_mass 

full_xcg = (nose_mass*nose_xcg + GNC_mass*GNC_xcg + warhead_mass*warhead_xcg + full_fuel_mass*full_fuel_xcg + front_mass*front_xcg + rear_mass*rear_xcg)/missile_mass_full
empty_xcg = (nose_mass*nose_xcg + GNC_mass*GNC_xcg + warhead_mass*warhead_xcg + empty_fuel_mass*empty_fuel_xcg + front_mass*front_xcg + rear_mass*rear_xcg)/missile_mass_empty

cg_shift = full_xcg - empty_xcg

r = diameter_L / 2;

% === Local Inertias (about their own CGs) ===
Iyy_nose_local    = (3/20) * nose_mass * r^2;
Iyy_GNC_local     = (1/12) * GNC_mass * (3*r^2 + GNC_L^2);
Iyy_warhead_local = (1/12) * warhead_mass * (3*r^2 + warhead_L^2);
% Iyy_fuel_local    = (1/12) * full_fuel_mass * (3*r^2 + full_fuel_L^2);
Iyy_fuel_empty    = (1/12) * empty_fuel_mass * (3*r^2 + full_fuel_L^2);

% === Rear Fin Approximations ===
% Treat each fin as a thin triangle about centroid: I = (1/36)*m*(a^2 + b^2)
a_r = rear_chord; b_r = rear_span; m_r = rear_mass / 4;
Iyy_rear_fins_local = 4 * (1/36) * m_r * (a_r^2 + b_r^2);

% === Distances from CG ===
d_nose    = abs(nose_xcg - empty_xcg);
d_GNC     = abs(GNC_xcg - empty_xcg);
d_warhead = abs(warhead_xcg - empty_xcg);
d_fuel    = abs(empty_fuel_xcg - empty_xcg);
d_front   = abs(front_xcg - empty_xcg);
d_rear    = abs(rear_xcg - empty_xcg);

% === Total Iyy using Parallel Axis Theorem ===
Iyy_total = ...
    Iyy_nose_local    + nose_mass    * d_nose^2 + ...
    Iyy_GNC_local     + GNC_mass     * d_GNC^2 + ...
    Iyy_warhead_local + warhead_mass * d_warhead^2 + ...
    Iyy_fuel_empty    + empty_fuel_mass * d_fuel^2 + ...
    Iyy_front_total + front_mass * d_front^2 + ...
    Iyy_rear_fins_local  + rear_mass  * d_rear^2;

fprintf('Total Iyy (about CG) = %.4f kg·m²\n', Iyy_total);



total_pixel = 1800;
total_length = 300;
in_per_pixel = total_pixel /total_length;









