% clear; clc; close all;

% === Geometry in inches ===
diameter_L = 12;  % in
Sref = pi * (diameter_L)^2 / 4;

nose_L = 36;      % in
nose_rho = 0.1;   % lb/in³
nose_xcg = (2/3) * nose_L;

GNC_L = 36;       
GNC_rho = 0.1;    
GNC_xcg = GNC_L/2 + nose_xcg;

warhead_L = 72;   
warhead_rho = 0.13; 
warhead_xcg = warhead_L/2 + GNC_xcg;

empty_full_L = 96;
empty_fuel_rho = 0.06;
empty_fuel_xcg = empty_full_L/2 + warhead_xcg;

full_fuel_L = 96;
full_fuel_rho = 0.11;
full_fuel_xcg = full_fuel_L/2 + warhead_xcg;

% === Fin Geometry in inches ===
rho_aluminum = 0.12;         % lb/in³
fin_thickness = 0.2;         % in

front_chord = 72;            % in
front_span = 24;             % in
front_root = 84;             % in
front_area = 0.5 * front_chord * front_span;
front_volume = front_area * fin_thickness;
front_mass = front_volume * rho_aluminum * 4;
front_xcg = front_root + front_chord / 3;

rear_chord = 24;             % in
rear_span = 24;              % in
rear_root = 216;             % in
rear_area = 0.5 * rear_chord * rear_span;
rear_volume = rear_area * fin_thickness;
rear_mass = rear_volume * rho_aluminum * 4;
rear_xcg = rear_root + rear_chord / 3;

% === Volumes & Masses ===
r = diameter_L / 2;  % in

nose_volume = (1/3) * pi * r^2 * nose_L;
nose_mass = nose_volume * nose_rho;

GNC_volume = pi * r^2 * GNC_L;
GNC_mass = GNC_volume * GNC_rho;

warhead_volume = pi * r^2 * warhead_L;
warhead_mass = warhead_volume * warhead_rho;

full_fuel_volume = pi * r^2 * full_fuel_L;
full_fuel_mass = full_fuel_volume * full_fuel_rho;

empty_fuel_volume = pi * r^2 * empty_full_L;
empty_fuel_mass = empty_fuel_volume * empty_fuel_rho;

fuel_mass = full_fuel_mass - empty_fuel_mass;

% === Mass & CG ===
missile_mass_full = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + full_fuel_mass;
missile_mass_empty = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + empty_fuel_mass;

full_xcg = (nose_mass * nose_xcg + GNC_mass * GNC_xcg + warhead_mass * warhead_xcg + ...
            full_fuel_mass * full_fuel_xcg + front_mass * front_xcg + rear_mass * rear_xcg) / missile_mass_full;

empty_xcg = (nose_mass * nose_xcg + GNC_mass * GNC_xcg + warhead_mass * warhead_xcg + ...
            empty_fuel_mass * empty_fuel_xcg + front_mass * front_xcg + rear_mass * rear_xcg) / missile_mass_empty;

cg_shift = full_xcg - empty_xcg;

% === Local Inertias (about component CGs) in lb·in² ===
Iyy_nose_local    = (3/20) * nose_mass * r^2;
Iyy_GNC_local     = (1/12) * GNC_mass * (3*r^2 + GNC_L^2);
Iyy_warhead_local = (1/12) * warhead_mass * (3*r^2 + warhead_L^2);
Iyy_fuel_empty    = (1/12) * empty_fuel_mass * (3*r^2 + full_fuel_L^2);

% === Fins ===
a_f = front_chord; b_f = front_span; m_f = front_mass / 4;
Iyy_front_fins_local = 4 * (1/36) * m_f * (a_f^2 + b_f^2);

a_r = rear_chord; b_r = rear_span; m_r = rear_mass / 4;
Iyy_rear_fins_local = 4 * (1/36) * m_r * (a_r^2 + b_r^2);

% === Distances from Empty CG ===
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
    Iyy_front_fins_local + front_mass * d_front^2 + ...
    Iyy_rear_fins_local  + rear_mass  * d_rear^2;

% === Output ===
full_xcg = full_xcg / 12;
fprintf('Full CG = %.2f ft\n', full_xcg);
empty_xcg = empty_xcg / 12;
fprintf('Empty CG = %.2f ft\n', empty_xcg);
cg_shift = cg_shift / 12;
fprintf('CG Shift = %.2f ft\n', cg_shift);
Iyy_total_lbft2 = Iyy_total / 144;
fprintf('Total Iyy (about Empty CG) = %.2f lb·ft²\n', Iyy_total_lbft2);
