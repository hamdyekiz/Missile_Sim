
% ====================================================================================
% MISSILE SIMULATION WITH GUIDANCE 
% This is a 3-DOF missile simulation with proportional navigation guidance
% and three-loop autopilot
% Authors: Burak Tasci, 
%          Furkan Balvan
%          Hamdi Ekiz
%====================================================================================

tic; % Start timing the simulation
profile off % Turn off MATLAB profiler

% Clear workspace and command window
clear
close all
clc
clearvars

global forces_moment aero_prop missile;

% Initialize main data structures
missile = struct();      % Missile parameters and properties
constants = struct();    % Physical constants and unit conversions
target = struct();       % Target parameters
thrust = struct();       % Propulsion system parameters
sim = struct();          % Simulation parameters
coef_data = struct();    % Aerodynamic coefficient data
sim_data = struct();     % Simulation output data

% ====================================================================================
% USEFUL CONSTANTS - Unit conversions and physical constants
% ====================================================================================
constants.d2r             = pi/180;        % Degrees to radians conversion
constants.r2d             = 180/pi;        % Radians to degrees conversion
constants.m2ft            = 3.28084;       % Meters to feet conversion
constants.kg2slug         = 0.0685218;     % Kilograms to slugs conversion
constants.kg2lb           = 2.20462;       % Kilograms to pounds conversion
constants.kgm2_to_lbft2   = 23.73036;      % kg⋅m² to lb⋅ft² conversion
constants.ft2m            = 0.3048;        % Feet to meters conversion
constants.slug2kg         = 14.5939;       % Slugs to kilograms conversion
constants.lb2kg           = 0.453592;      % Pounds to kilograms conversion
constants.lbft2_to_kgm2   = 0.0421401;     % lb⋅ft² to kg⋅m² conversion
constants.g               = 9.81;          % Gravitational acceleration [m/s²]

% ====================================================================================
% USER INPUTS: TARGET PARAMETERS
% ====================================================================================
R_Tx = 8000;                          % Initial target position X-component [m]
R_Tz = 8000;                          % Initial target position Z-component [m]
target.V_T = 300;                     % Target velocity magnitude [m/s]
Beta = -15;                           % Target velocity angle [degrees]
target.XNT = -2*constants.g;          % Target maneuver acceleration [m/s²]

% ====================================================================================
% USER INPUTS: MISSILE PARAMETERS
% ====================================================================================
R_Mx = 0;                             % Initial missile position X-component [m]
R_Mz = 1000;                          % Initial missile position Z-component [m]
V_M = 500;                            % Initial missile velocity magnitude [m/s]
q = 0;                                % Initial pitch rate [rad/s]
HE = -5;                              % Head error angle [degrees]
missile.XNP = 5;                      % Effective navigation ratio (proportional navigation gain)

% ====================================================================================
% MISSILE GEOMETRIC PROPERTIES
% ====================================================================================
missile.lref = 0.3048;                % Missile reference length (diameter) [m]
missile.S = pi * missile.lref^2 / 4;  % Missile reference area [m²]

% Compute missile mass properties from geometric calculations
[missile.mass, missile.fuel_mass, missile.xcg, missile.Iyy] = compute_missile_geometric_properties();
missile.dry_mass = missile.mass - missile.fuel_mass; % Mass without fuel [kg]

% ====================================================================================
% THRUST PARAMETERS
% ====================================================================================
% Specific impulse values for different propellants:
% Solid propellant    = 180-270 sec
% N204/MMH           = 260-310 sec
% N204/UDMH          = 260-310 sec
% Kerosene/LOX       = 300-350 sec
% LH2/LOX            = 455 sec
% LH2/LF             = 475 sec
thrust.I_sp     = 300;                                                             % Specific impulse [sec] 
thrust.t_b      = 5;                                                               % Burn time [sec]
thrust.T_total  = thrust.I_sp * constants.g * (missile.fuel_mass / thrust.t_b);    % Total thrust magnitude [N]

% ====================================================================================
% INITIAL CALCULATIONS: ENGAGEMENT GEOMETRY
% ====================================================================================

% Convert input angles from degrees to radians
Beta = Beta * pi/180;           % Target velocity angle [rad]
HE_rad = HE * pi/180;          % Head error angle [rad]

% Calculate initial line-of-sight (LOS) geometry
R_TMx = R_Tx - R_Mx;                      % LOS X-component [m]
R_TMz = R_Tz - R_Mz;                      % LOS Z-component [m]
R_TM = sqrt(R_TMx*R_TMx + R_TMz*R_TMz);   % LOS distance [m]

% Calculate engagement angles
Lamda = atan2(R_TMz , R_TMx);             % Line-of-sight angle [rad]
Lead = asin(target.V_T*sin(Beta+Lamda)/V_M);     % Missile lead angle for intercept [rad]

% Calculate initial target velocity components
V_Tx = -target.V_T * cos(Beta);           % Target velocity X-component [m/s]
V_Tz =  target.V_T * sin(Beta);           % Target velocity Z-component [m/s]

% Calculate initial missile heading angle
theta = Lamda + Lead + HE_rad;            % Missile pitch angle [rad]

% Calculate initial missile velocity components
V_Mx = V_M * cos(theta);                  % Missile velocity X-component [m/s]
V_Mz = V_M * sin(theta);                  % Missile velocity Z-component [m/s]

% Calculate relative velocity components
V_TMx = V_Tx - V_Mx;                      % Relative velocity X-component [m/s]
V_TMz = V_Tz - V_Mz;                      % Relative velocity Z-component [m/s]
V_C = -(R_TMx * V_TMx + R_TMz * V_TMz) / R_TM;  % Closing velocity [m/s] (positive = closing)

% ====================================================================================
% READ AERODYNAMIC AND ATMOSPHERIC DATA TABLES
% ====================================================================================
% Load aerodynamic coefficient data as functions of XCG, ALPHA, MACH, DELTA
[XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, AERO_grid] = read_coef();

% Load atmospheric data (altitude vs air speed and density)
[alt_table, V_a_table, rho_table, V_air_interp, rho_interp] = read_air_data();




% ====================================================================================
% SIMULATION INITIALIZATION
% ====================================================================================
sim.t = 0.;                               % Initial simulation time [s]
sim.n = 0.;                               % Simulation step counter
sim.h = 0;                                % Integration step size [s]
sim.flag = false;                         % Simulation completion flag

% Initialize state vector: [R_Tx, R_Tz, V_Tx, V_Tz, Beta, R_Mx, R_Mz, V_Mx, V_Mz, theta, q]
X_s = [R_Tx; R_Tz; V_Tx; V_Tz; Beta; R_Mx; R_Mz; V_Mx; V_Mz; theta; q];

L = length(X_s);                          % Length of state vector

% ====================================================================================
% MAIN SIMULATION LOOP
% Integration using Runge-Kutta 4th Order Method
% ====================================================================================
while V_C >= 0  && R_TM >= 0
    
    % Adaptive step size based on range to target
    if R_TM < 500 
        sim.h = 0.001;                    % Small step size for close-range [s]
    else
        sim.h = 0.001;                    % Standard step size [s]
    end

    % Advance time and step counter
    sim.t = sim.t + sim.h;                % Update simulation time [s]
    sim.n = sim.n + 1;                    % Increment step counter
  
    % ===== SIMULATION STEP =====
    % Integrate equations of motion using 4th-order Runge-Kutta
    X_s = RK4(X_s, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);
    % ===== END SIMULATION STEP =====

    % Extract updated state variables from state vector
    R_Tx    = X_s(1);                     % Target position X [m]
    R_Tz    = X_s(2);                     % Target position Z [m]
    Beta    = X_s(5);                     % Target velocity angle [rad]
    R_Mx    = X_s(6);                     % Missile position X [m]
    R_Mz    = X_s(7);                     % Missile position Z [m]
    V_Mx    = X_s(8);                     % Missile velocity X [m/s]
    V_Mz    = X_s(9);                     % Missile velocity Z [m/s]
    theta   = X_s(10);                    % Missile pitch angle [rad]
    q       = X_s(11);                    % Missile pitch rate [rad/s]

    % Calculate angle of attack in body-fixed coordinate system
    [u_b, w_b] = transform(V_Mx, V_Mz, theta, "I2B");  % Transform to body coordinates
    alpha = atan2(w_b, u_b)*180/pi;       % Angle of attack [degrees]

    % Update target velocity components
    X_s(3) = -target.V_T*cos(Beta);       % Target velocity X-component [m/s]
    X_s(4) =  target.V_T*sin(Beta);       % Target velocity Z-component [m/s]
    V_Tx = X_s(3);                       
    V_Tz = X_s(4);                       
    
    % Recompute engagement geometry for current state
    R_TMx = R_Tx - R_Mx;                  % LOS X-component [m]
    R_TMz = R_Tz - R_Mz;                  % LOS Z-component [m]
    R_TM = sqrt(R_TMx^2 + R_TMz^2);       % LOS distance [m]
    V_TMx = V_Tx - V_Mx;                  % Relative velocity X [m/s]
    V_TMz = V_Tz - V_Mz;                  % Relative velocity Z [m/s]
    Lamda = atan2(R_TMz, R_TMx);          % LOS angle [rad]
    Lamda_d = (V_TMz * R_TMx - V_TMx * R_TMz) / (R_TM^2);  % LOS angle rate [rad/s]
    V_C = -(R_TMx * V_TMx + R_TMz * V_TMz) / R_TM;         % Closing velocity [m/s]
    XNC = missile.XNP * V_C * Lamda_d;    % Commanded acceleration from proportional navigation [m/s²]
    
    % Print simulation status at regular intervals or near impact
    if mod(sim.n, 1000) == 0 || R_TM < 0.5 || V_C < 1e-3    
        fprintf('Step %d, Time %.4f sec\n', sim.n, sim.t);
        fprintf('alpha  = %.4f deg\n', atan2(w_b, u_b) * 180/pi);
        fprintf('x      = %.4f m\n', X_s(6));
        fprintf('z      = %.4f m\n', X_s(7));
        fprintf('u      = %.4f m/s\n', X_s(8));
        fprintf('w      = %.4f m/s\n', X_s(9));
        fprintf('theta  = %.4f deg\n', X_s(10)*180/pi);
        fprintf('q      = %.4f deg/s\n', X_s(11)*180/pi);
        fprintf('----------------------------------\n');   
    end

    % Save simulation data for post-processing
    save_data(sim, X_s, missile, forces_moment, aero_prop, R_TMx, R_TMz, V_TMx, V_TMz, Lamda, Lamda_d, V_C, XNC, alpha);

end % End main simulation loop

% Print intercept message
fprintf('Missile hit the target at %.2f s\n', sim.t);
sim.flag = true;  % Set completion flag

% ====================================================================================
% POST-PROCESSING: SAVE AND PLOT RESULTS
% ====================================================================================

% Get final saved data structure
saved_data = save_data(sim, X_s, missile, forces_moment, aero_prop, R_TMx, R_TMz, V_TMx, V_TMz, Lamda, Lamda_d, V_C, XNC, alpha);

% Generate plots of simulation results
plot_data(saved_data);

% Export data to CSV files
export_saved_data_to_csv(saved_data);

% Stop timing and display elapsed time
elapsed_time = toc;
fprintf('\n Simulation completed in %.3f seconds.\n', elapsed_time);

% profile viewer


% ====================================================================================
% MISSILE DYNAMICS FUNCTION
% Computes derivatives of state vector for numerical integration
% ====================================================================================
function dX = missile_dynamics(X, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp)
    global forces_moment aero_prop missile;
    
    % Extract state variables
    R_Tx    = X(1);     % Target position X [m]
    R_Tz    = X(2);     % Target position Z [m]
    Beta    = X(5);     % Target velocity angle [rad]
    R_Mx    = X(6);     % Missile position X [m]
    R_Mz    = X(7);     % Missile position Z [m]
    V_Mx    = X(8);     % Missile velocity X [m/s]
    V_Mz    = X(9);     % Missile velocity Z [m/s]
    theta   = X(10);    % Missile pitch angle [rad]
    theta_d = X(11);    % Missile pitch rate [rad/s]
 
    % Calculate target velocity components
    V_Tx    = -target.V_T*cos(Beta);      % Target velocity X-component [m/s]
    V_Tz    =  target.V_T*sin(Beta);      % Target velocity Z-component [m/s]

    % Calculate engagement geometry
    R_TMx = R_Tx - R_Mx;                  % LOS X-component [m]
    R_TMz = R_Tz - R_Mz;                  % LOS Z-component [m]
    R_TM = sqrt(R_TMx*R_TMx + R_TMz*R_TMz);  % LOS distance [m]
    V_TMx = V_Tx - V_Mx;                  % Relative velocity X [m/s]
    V_TMz = V_Tz - V_Mz;                  % Relative velocity Z [m/s]
    V_C = -(R_TMx*V_TMx + R_TMz*V_TMz)/R_TM;  % Closing velocity [m/s]
    Lamda = atan2(R_TMz,R_TMx);           % LOS angle [rad]
    Lamda_d = (V_TMz*R_TMx - V_TMx*R_TMz)/(R_TM*R_TM);  % LOS angle rate [rad/s]
    
    % Proportional navigation guidance law
    XNC = missile.XNP*V_C*Lamda_d;        % Commanded acceleration [m/s²]
    A_Mx = -XNC*sin(Lamda);               % Commanded acceleration X-component [m/s²]
    A_Mz =  XNC*cos(Lamda);               % Commanded acceleration Z-component [m/s²]
    
    % Target maneuver dynamics
    Beta_d = target.XNT/target.V_T;       % Target velocity angle rate [rad/s]

    % Update missile mass due to fuel consumption
    if sim.t <= thrust.t_b
        % Linear fuel consumption during burn time
        current_fuel_mass = missile.fuel_mass * (1 - sim.t / thrust.t_b);
        missile.mass = missile.dry_mass + current_fuel_mass;
        % Update center of gravity and moment of inertia
        [missile.xcg, missile.Iyy] = update_xcg_Iyy(current_fuel_mass);
        % Compute thrust force (decreases linearly with fuel)
        T = thrust_force(sim.t, thrust);
    else
        % After burnout: no fuel, no thrust
        [missile.xcg, missile.Iyy] = update_xcg_Iyy(0);
        missile.mass = missile.dry_mass;
        T = 0;
    end

    % Debugging breakpoint 
    % if sim.t >= 2.249
    %     keyboard;  
    % end   

    % Calculate aerodynamic forces and moments
    % Transform velocity to body-fixed coordinates
    [u_B, w_B] = transform(V_Mx, V_Mz, theta, "I2B");
    alpha = atan2(w_B, u_B);              % Angle of attack [rad]
    V = sqrt(u_B^2 + w_B^2);              % Total velocity magnitude [m/s]
    delta = -1;                           % Control surface deflection [deg] (fixed)
    
    % Get aerodynamic derivatives from lookup tables
    [aero_derivatives, rho, MACH] = get_derivatives(missile.xcg, alpha * 180 / pi, V, R_Mz, delta, AERO_grid, V_air_interp, rho_interp);
    CN = aero_derivatives(1);             % Normal force coefficient
    CA = aero_derivatives(3);             % Axial force coefficient

    % Calculate dynamic pressure
    Q = 0.5*rho*V^2;                      % Dynamic pressure [Pa]
    
    % Calculate aerodynamic forces in body coordinates
    [Lf, Df, X_force, Z_force] = calculate_aero_forces(CN, CA, Q, missile.S);

    % Calculate missile accelerations in body coordinates
    ud_B = (X_force + T) / missile.mass - constants.g*sin(theta);  % Axial acceleration [m/s²]
    wd_B = Z_force / missile.mass - constants.g*cos(theta);        % Normal acceleration [m/s²]
    
    % Transform accelerations to inertial coordinates
    [ud_In, wd_In] = transform(ud_B, wd_B, theta, "B2I");

    % Add guidance commands to accelerations
    ud_I = ud_In + A_Mx;                  % Total acceleration X [m/s²]
    wd_I = wd_In + A_Mz;                  % Total acceleration Z [m/s²]

    % Calculate pitching moment and angular acceleration
    CM  = aero_derivatives(2);            % Pitching moment coefficient
    M = Q * missile.S * missile.lref* CM; % Pitching moment [N⋅m]
    theta_dd = -M / missile.Iyy;          % Angular acceleration [rad/s²]

    % Store forces and moments for data logging
    forces_moment = struct();         
    forces_moment.Lf = Lf;                % Lift force [N]
    forces_moment.Df = Df;                % Drag force [N]
    forces_moment.X_force = X_force;      % X-direction force [N]
    forces_moment.Z_force = Z_force;      % Z-direction force [N]
    forces_moment.T = T;                  % Thrust force [N]
    forces_moment.M = M;                  % Pitching moment [N⋅m]

    % Store aerodynamic properties for data logging
    aero_prop = struct();         
    aero_prop.Q = Q;                      % Dynamic pressure [Pa]
    aero_prop.rho = rho;                  % Air density [kg/m³]
    aero_prop.aero_derivatives = aero_derivatives;  % Aerodynamic coefficients
    
    missile.MACH = MACH;                  % Store Mach number

    % Assemble derivative vector for integration
    % dX = [dR_Tx/dt, dR_Tz/dt, dV_Tx/dt, dV_Tz/dt, dBeta/dt, dR_Mx/dt, dR_Mz/dt, dV_Mx/dt, dV_Mz/dt, dtheta/dt, dq/dt]
    dX = [V_Tx; V_Tz; 0; 0; Beta_d; V_Mx; V_Mz; ud_I; wd_I; theta_d; theta_dd;];
             
end

% ====================================================================================
% RUNGE-KUTTA 4TH ORDER INTEGRATION
% Numerical integration scheme for solving differential equations
% ====================================================================================
function X_s = RK4(X_s, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp)

    X_tmp = X_s;
    K1 = missile_dynamics(X_tmp, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);

    X_tmp = X_s;
    K2 = missile_dynamics(X_tmp + 0.5 * sim.h * K1, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);

    X_tmp = X_s;
    K3 = missile_dynamics(X_tmp + 0.5 * sim.h * K2, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);

    X_tmp = X_s;
    K4 = missile_dynamics(X_tmp + sim.h * K3, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);

    X_s = X_s + (sim.h / 6) * (K1 + 2*K2 + 2*K3 + K4);

end




% ====================================================================================
% CALCULATE AERODYNAMIC FORCES
% Convert aerodynamic coefficients to force components
% ====================================================================================
function[Lf, Df, X_force, Z_force] = calculate_aero_forces(CN, CA, Q, S)

    % Compute aerodynamic forces from coefficients
    Df = Q * S * CA;                     % Drag force [N] 
    Lf = Q * S * CN;                     % Lift force [N] 

    % Force components in body-fixed coordinate system
    X_force = -Df;                       % X-direction force [N] 
    Z_force = -Lf;                       % Z-direction force [N]
end

% ====================================================================================
% THRUST FORCE CALCULATION
% Computes thrust as function of time 
% ====================================================================================
function T = thrust_force(t, thrust)
    if t <= thrust.t_b
        % Thrust decreases linearly as fuel is consumed
        T = thrust.T_total * (1 - t / thrust.t_b);  % [N]
    else
        % No thrust after burnout
        T = 0;                           % [N]
    end
end

% ====================================================================================
% COORDINATE TRANSFORMATION
% Transform vectors between inertial and body-fixed coordinate systems
% ====================================================================================
function [u_t, w_t] = transform(u, w, theta, mode)

    if mode == "I2B"  % Inertial to Body transformation
        u_t =  cos(theta) * u + sin(theta) * w;   % Body X-component
        w_t = -sin(theta) * u + cos(theta) * w;   % Body Z-component
    elseif mode == "B2I"  % Body to Inertial transformation
        u_t =  cos(theta) * u - sin(theta) * w;   % Inertial X-component
        w_t =  sin(theta) * u + cos(theta) * w;   % Inertial Z-component
    else
        error("Mode must be 'I2B' or 'B2I'");
    end
end




% ====================================================================================
% PLOT SIMULATION RESULTS
% Generate comprehensive plots of missile trajectory and performance
% ====================================================================================
function plot_data(saved_data)

    % Target and Missile Path
    figure;
    hold on;
    
    plot(saved_data.x_t_all, saved_data.z_t_all, 'r--', 'LineWidth', 2);  % Target path
    plot(saved_data.x_all, saved_data.z_all, 'b-', 'LineWidth', 2);       % Missile path
    
    xlabel('X Position (m)');
    ylabel('Z Position (m)');
    title('Target and Missile Path');
    legend('Target Path', 'Missile Path', 'Location', 'best');
    grid on;

    
    % Angle of Attack
    figure;
    plot(saved_data.time_all, saved_data.alpha_all, 'b'); 
    xlabel('Time (s)'); ylabel('Alpha (deg)');
    title('Angle of Attack Over Time');
    grid on;

    % Velocity and Components
    figure;
    % u Component
    subplot(3,1,1);
    plot(saved_data.time_all, saved_data.u_all, 'r');
    ylabel('u (m/s)');
    title('Velocity Component u Over Time');
    grid on;

    % w Component
    subplot(3,1,2);
    plot(saved_data.time_all, saved_data.w_all, 'b');
    ylabel('w (m/s)');
    title('Velocity Component w Over Time');
    grid on;

    % Total Velocity V
    subplot(3,1,3);
    plot(saved_data.time_all, saved_data.V_all, 'k');
    ylabel('V (m/s)');
    xlabel('Time (s)');
    title('Total Velocity Over Time');
    grid on;


    % Pitch Angle and Pitch Rate
    figure;
    % Pitch Angle
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.theta_all * 180/pi, 'b', 'LineWidth', 1.5);
    ylabel('\theta (deg)');
    title('Pitch Angle Over Time');
    grid on;
    % Pitch Rate
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.thetadot_all * 180/pi, 'r', 'LineWidth', 1.5);
    ylabel('q (deg/s)');
    xlabel('Time (s)');
    title('Pitch Rate Over Time');
    grid on;

    % Closing Velocity and Commanded Acceleration
    figure;
    % Closing Velocity
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.V_C_all, 'm', 'LineWidth', 1.5);
    ylabel('V_C (m/s)');
    title('Closing Velocity Over Time');
    grid on;
    % Commanded Acceleration
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.XNC_all, 'k', 'LineWidth', 1.5);
    ylabel('XNC (g)'); xlabel('Time (s)');
    title('Commanded Acceleration Over Time');
    grid on;

    %  Line-of-Sight Angle and Rate 
    figure;
    % Lambda
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.Lamda_all * 180/pi, 'b', 'LineWidth', 1.5);
    ylabel('\lambda (deg)');
    title('Line-of-Sight Angle Over Time');
    grid on;
    % Lambda_dot
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.Lamda_d_all * 180/pi, 'r', 'LineWidth', 1.5);
    ylabel('d\lambda/dt (deg/s)');
    xlabel('Time (s)');
    title('LOS Angle Rate Over Time');
    grid on;


    % Air Density and Dynamic Pressure
    figure;
    % Air Density
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.rho_all, 'b', 'LineWidth', 1.5);
    ylabel('Density (kg/m³)');
    title('Air Density Over Time');
    grid on;

    % Dynamic Pressure
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.Q_all, 'r', 'LineWidth', 1.5);
    ylabel('Q (Pa)');
    xlabel('Time (s)');
    title('Dynamic Pressure Over Time');
    grid on;

    % Forces
    figure;
    % Lift Force
    subplot(2,2,1);
    plot(saved_data.time_all, saved_data.Lf_all, 'r');
    ylabel('Lift Force');
    title('Lift Force Over Time');
    grid on;
    % Z Force
    subplot(2,2,3);
    plot(saved_data.time_all, saved_data.Z_force_all, 'b');
    ylabel('Z Force');
    xlabel('Time (s)');
    title('Z Force Over Time');
    grid on;
    % Drag Force
    subplot(2,2,2);
    plot(saved_data.time_all, saved_data.Df_all, 'r');
    ylabel('Drag Force');
    title('Drag Force Over Time');
    grid on;
    % X Force
    subplot(2,2,4);
    plot(saved_data.time_all, saved_data.X_force_all, 'b');
    ylabel('X Force');
    xlabel('Time (s)');
    title('X Force Over Time');
    grid on;

    % Moment
    figure;
    plot(saved_data.time_all, saved_data.M_all, 'b'); 
    xlabel('Time (s)'); ylabel('Moment');
    title('Moment Over Time');
    grid on;

    %  Aerodynamic Coefficients 
    figure;
    % CM
    subplot(3,2,1);
    plot(saved_data.time_all, saved_data.CM_all, 'b'); 
    xlabel('Time (s)'); ylabel('CM');
    title('CM Over Time');
    grid on;
    % CMA
    subplot(3,2,3);
    plot(saved_data.time_all, saved_data.CMA_all, 'b'); 
    xlabel('Time (s)'); ylabel('CMA');
    title('CMA Over Time');
    grid on;
    % CMQ
    subplot(3,2,5);
    plot(saved_data.time_all, saved_data.CMQ_all, 'b'); 
    xlabel('Time (s)'); ylabel('CMQ');
    title('CMQ Over Time');
    grid on;
    % CL
    subplot(3,2,2);
    plot(saved_data.time_all, saved_data.CL_all, 'b'); 
    xlabel('Time (s)'); ylabel('CL');
    title('CL Over Time');
    grid on;
    % CD
    subplot(3,2,4);
    plot(saved_data.time_all, saved_data.CD_all, 'b'); 
    xlabel('Time (s)'); ylabel('CD');
    title('CD Over Time');
    grid on;
    % CA
    subplot(3,2,6);
    plot(saved_data.time_all, saved_data.CA_all, 'b'); 
    xlabel('Time (s)'); ylabel('CA');
    title('CA Over Time');
    grid on;
end

% ====================================================================================
% READ AERODYNAMIC COEFFICIENT DATA
% Loads aerodynamic coefficient lookup tables from CSV files
% ====================================================================================
function [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, AERO_grid] = read_coef()
    folderPath = fullfile(pwd, 'coef_output_def');
    csvFiles = dir(folderPath);
    csvFiles = csvFiles(3:end);  
    N_xcg = length(csvFiles);
    xcgFiles = strings(N_xcg, 1);
    
    for i = 1:N_xcg
        xcgFiles(i) = fullfile(csvFiles(i).folder, csvFiles(i).name);
    end

    alphaFilesStruct = dir(fullfile(xcgFiles(1), 'alpha*.csv'));
    N_alpha = length(alphaFilesStruct);

    % Coefficient fields to extract
    fields = {'CN','CM','CA','CL','CD','CMA','CNA','CMQ','CNQ'};

    % Preallocate containers
    alpha_vals = zeros(N_alpha, 1);
    T0 = readtable(fullfile(alphaFilesStruct(1).folder, alphaFilesStruct(1).name));
    mach_vals = unique(T0.MACH);
    delta_vals = unique(T0.DELTA);

    N_mach = length(mach_vals);
    N_delta = length(delta_vals);

    % Preallocate coefficient data
    data_grid = struct.empty(N_xcg, 0);
    for i = 1:N_xcg
        for f = 1:length(fields)
            data_grid(i).(fields{f}) = nan(N_alpha, N_mach, N_delta);
        end
    end

    xcg_vals = zeros(N_xcg, 1);

    % === Read data ===
    for i = 1:N_xcg
        alphaFilesStruct = dir(fullfile(xcgFiles(i), 'alpha*.csv'));
        for k = 1:N_alpha
            fullFileName = fullfile(alphaFilesStruct(k).folder, alphaFilesStruct(k).name);
            T = readtable(fullFileName);

            alpha_vals(k) = T.ALPHA(1);
            xcg_vals(i) = T.XCG(1);

            for f = 1:length(fields)
                coeff_mat = nan(N_mach, N_delta);
                for m = 1:N_mach
                    for d = 1:N_delta
                        idx = T.MACH == mach_vals(m) & T.DELTA == delta_vals(d);
                        val = T.(fields{f})(idx);
                        if ~isempty(val)
                            coeff_mat(m,d) = val(1);
                        end
                    end
                end
                data_grid(i).(fields{f})(k, :, :) = coeff_mat;
            end
        end
    end

    % === Sort alpha and xcg ===
    [alpha_vals, alpha_sort_idx] = sort(alpha_vals);
    [xcg_vals, xcg_sort_idx] = sort(xcg_vals);

    for f = 1:length(fields)
        for i = 1:N_xcg
            data_grid(i).(fields{f}) = data_grid(i).(fields{f})(alpha_sort_idx, :, :);
        end
    end

    % === Create ndgrid ===
    [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid] = ndgrid(xcg_vals, alpha_vals, mach_vals, delta_vals);

    % === Compute CND and CMD using central difference ===
    CND_grid = nan(N_xcg, N_alpha, N_mach, N_delta);
    CMD_grid = nan(N_xcg, N_alpha, N_mach, N_delta);

    for i = 1:N_xcg
        CN = squeeze(data_grid(i).CN); % [N_alpha x N_mach x N_delta]
        CM = squeeze(data_grid(i).CM);

        for k = 1:N_alpha
            for m = 1:N_mach
                for d = 1:N_delta
                    delta = delta_vals(d);

                    if d == 1 || d == N_delta
                        CND_grid(i,k,m,d) = NaN;
                        CMD_grid(i,k,m,d) = NaN;
                        continue;
                    end

                    d_plus = delta_vals(d+1);
                    d_minus = delta_vals(d-1);

                    CN_plus = CN(k,m,d+1);
                    CN_minus = CN(k,m,d-1);

                    CM_plus = CM(k,m,d+1);
                    CM_minus = CM(k,m,d-1);

                    % Check for NaNs before calculating
                    if all(~isnan([CN_plus, CN_minus, d_plus, d_minus]))
                        CND_grid(i,k,m,d) = (CN_plus - CN_minus) / (d_plus - d_minus);
                    end

                    if all(~isnan([CM_plus, CM_minus, d_plus, d_minus]))
                        CMD_grid(i,k,m,d) = (CM_plus - CM_minus) / (d_plus - d_minus);
                    end
                end
            end
        end
    end

    % === Build interpolants ===
    AERO_grid = struct();

    for f = 1:length(fields)
        all_matrix = nan(N_xcg, N_alpha, N_mach, N_delta);
        for i = 1:N_xcg
            all_matrix(i,:,:,:) = data_grid(i).(fields{f});
        end
        AERO_grid.(fields{f}) = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, all_matrix, 'linear', 'none');
    end

    % Add control derivatives (CND, CMD) to AERO_grid
    AERO_grid.CND = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, CND_grid, 'linear', 'none');
    AERO_grid.CMD = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, CMD_grid, 'linear', 'none');
end

% ====================================================================================
% READ ATMOSPHERIC DATA
% Parse air velocity and density tables for atmospheric modeling
% ====================================================================================
function [alt_table, V_a_table, rho_table, V_air_interp, rho_interp] = read_air_data()

    % Read the CSV file
    folderpath = fullfile(pwd, 'air_table.csv');
    data = readtable(folderpath);

    % Extract arrays (directly from table variables)
    alt_table = data.Altitude;    
    V_a_table = data.V_a;
    rho_table = data.rho;
    V_air_interp = griddedInterpolant(alt_table, V_a_table, 'linear', 'nearest');
    rho_interp = griddedInterpolant(alt_table, rho_table, 'linear', 'nearest');
end

% ====================================================================================
% INTERPOLATE AERODYNAMIC DERIVATIVES
% Get aerodynamic coefficients for current flight conditions
% ====================================================================================
function [aero, rho_val, mach_number] = get_derivatives(xcg, alpha, V, z, delta, AERO_grid, V_air_interp, rho_interp)
    V_air = V_air_interp(z);
    rho_val = rho_interp(z);
    mach_number = V / V_air;
    
    % Get aero coefficients including CND and CMD directly from interpolants
    fields = {'CN','CM','CA','CL','CD','CMA','CNA','CMQ','CNQ','CND','CMD'};
    res = zeros(length(fields), 1);

    for f = 1:length(fields)
        interp_func = AERO_grid.(fields{f});
        res(f) = interp_func(xcg, alpha, mach_number, delta);
    end
    
    aero = res;
end



% ====================================================================================
% SAVE SIMULATION DATA
% Accumulate data during simulation for post-processing and plotting
% ====================================================================================
function [saved_data] = save_data(sim, X_s, missile, forces_moment, aero_prop, R_TMx, R_TMz, V_TMx, V_TMz, Lamda, Lamda_d, V_C, XNC,alpha)
    persistent V_all M_all Lf_all Df_all X_force_all Z_force_all;
    persistent CM_all CL_all CD_all CN_all CA_all CMA_all CNA_all CMQ_all CNQ_all Q_all;
    persistent x_t_all z_t_all u_t_all w_t_all beta_all x_all z_all u_all w_all theta_all thetadot_all;
    persistent alpha_all rho_all time_all xcg_all mass_all MACH_all Iyy_all;
    persistent R_TMx_all R_TMz_all R_TM_all V_TMx_all V_TMz_all Lamda_all Lamda_d_all V_C_all XNC_all;
    
    % Initialize arrays on the first call
    if isempty(V_all)
        V_all = [];
        M_all = [];
        Lf_all = [];
        Df_all = [];
        X_force_all = [];
        Z_force_all = [];
        alpha_all = [];
        rho_all = [];
        time_all = [];   
        xcg_all = [];
        mass_all = [];
        Iyy_all = [];
        MACH_all = [];

        % Initialize aerodynamic coefficients arrays
        Q_all = [];
        CN_all = [];
        CM_all = [];
        CA_all = [];
        CL_all = [];
        CD_all = [];
        CMA_all = [];
        CNA_all = [];
        CMQ_all = [];
        CNQ_all = [];

        % Initialize state vector arrays
        x_t_all = [];
        z_t_all = [];     
        u_t_all = [];
        w_t_all = []; 
        beta_all = [];
        x_all = [];
        z_all = [];
        u_all = [];
        w_all = [];
        theta_all = [];
        thetadot_all = [];

        % Initialize guidance related parameters
        R_TMx_all = [];
        R_TMz_all = [];
        R_TM_all = [];
        V_TMx_all = [];
        V_TMz_all = [];
        Lamda_all = [];
        Lamda_d_all = [];
        V_C_all = [];
        XNC_all = [];
        
    end

    % Save data to arrays for the current step
    M_all(sim.n) = forces_moment.M;
    Lf_all(sim.n) = forces_moment.Lf;
    Df_all(sim.n) = forces_moment.Df;
    X_force_all(sim.n) = forces_moment.X_force;
    Z_force_all(sim.n) = forces_moment.Z_force;
    V_all(sim.n) = sqrt(X_s(8)^2 + X_s(9)^2);
    alpha_all(sim.n) = alpha;
    rho_all(sim.n) = aero_prop.rho;  
    xcg_all(sim.n) = missile.xcg;
    mass_all(sim.n) = missile.mass;
    Iyy_all(sim.n) = missile.Iyy;
    MACH_all(sim.n) = missile.MACH;    
    time_all(sim.n) = sim.t;

    % Save aerodynamic coefficients to arrays   
    Q_all(sim.n) = aero_prop.Q;              % Dynamic Pressure
    CN_all(sim.n) = aero_prop.aero_derivatives(1);     % Longitudinal Force Coefficient
    CM_all(sim.n) = aero_prop.aero_derivatives(2);     % Pitching Moment Coefficient
    CA_all(sim.n) = aero_prop.aero_derivatives(3);     % Axial Force Coefficient
    CL_all(sim.n) = aero_prop.aero_derivatives(4);     % Lift Coefficient
    CD_all(sim.n) = aero_prop.aero_derivatives(5);     % Drag Coefficient
    CMA_all(sim.n) = aero_prop.aero_derivatives(6);   % Derivative of Pitching Moment Coefficient with respect to Angle of Attack
    CNA_all(sim.n) = aero_prop.aero_derivatives(7);    
    CMQ_all(sim.n) = aero_prop.aero_derivatives(8)*missile.lref/(2*(sqrt(X_s(8)^2 + X_s(9)^2)));   % Derivative of Pitching Moment Coefficient with respect to pitch rate
    CNQ_all(sim.n) = aero_prop.aero_derivatives(9)*missile.lref/(2*(sqrt(X_s(8)^2 + X_s(9)^2)));

    % Save state vector data to arrays for the current step
    x_t_all(sim.n) = X_s(1);
    z_t_all(sim.n) = X_s(2);  
    u_t_all(sim.n) = X_s(3);
    w_t_all(sim.n) = X_s(4); 
    beta_all(sim.n) = X_s(5);
    x_all(sim.n) = X_s(6);
    z_all(sim.n) = X_s(7);
    u_all(sim.n) = X_s(8);
    w_all(sim.n) = X_s(9);
    theta_all(sim.n) = X_s(10);
    thetadot_all(sim.n) = X_s(11); 

    % Save guidance related parameters to arrays for the current step
    R_TMx_all(sim.n) = R_TMx;
    R_TMz_all(sim.n) = R_TMz;
    R_TM_all(sim.n) =  sqrt(R_TMx^2 + R_TMz^2);
    V_TMx_all(sim.n) = V_TMx;
    V_TMz_all(sim.n) = V_TMz;
    Lamda_all(sim.n) = Lamda;
    Lamda_d_all(sim.n) = Lamda_d;
    V_C_all(sim.n) = V_C;
    XNC_all(sim.n) = XNC / 9.81;

    if sim.flag
        saved_data = struct('V_all', V_all, 'M_all', M_all, 'Lf_all', Lf_all, ...
                            'Df_all', Df_all, 'X_force_all', X_force_all, ...
                            'Z_force_all', Z_force_all, 'CN_all', CN_all, ...
                            'CM_all', CM_all, 'CA_all', CA_all, 'CL_all', CL_all, 'CD_all', CD_all, ...
                            'CMA_all', CMA_all, 'CNA_all', CNA_all, 'CMQ_all', CMQ_all, 'CNQ_all', CNQ_all, 'Q_all', Q_all,...
                            'beta_all', beta_all, 'x_t_all', x_t_all, 'z_t_all', z_t_all,'u_t_all', u_t_all, 'w_t_all', w_t_all,...
                            'x_all', x_all, 'z_all', z_all, 'u_all',u_all,'w_all', w_all, ...
                            'theta_all', theta_all, 'thetadot_all', thetadot_all, 'alpha_all', alpha_all, 'rho_all', rho_all, ...
                            'time_all', time_all, 'R_TMx_all', R_TMx_all, 'R_TMz_all', R_TMz_all, 'R_TM_all', R_TM_all, ...
                            'V_TMx_all', V_TMx_all, 'V_TMz_all', V_TMz_all, 'Lamda_all', Lamda_all, 'Lamda_d_all', Lamda_d_all, ...
                            'V_C_all', V_C_all, 'XNC_all', XNC_all, 'xcg_all', xcg_all, 'mass_all', mass_all, 'Iyy_all', Iyy_all, 'MACH_all', MACH_all);
    end


end

% ====================================================================================
% ==== EXPORT DATA TO CSV FILE FORMAT ====
% ====================================================================================
function export_saved_data_to_csv(saved_data)

    folder_name = 'saved_data';
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);
    end

    fields = fieldnames(saved_data);
    time_vector = saved_data.time_all(:);  

    for i = 1:length(fields)
        field_name = fields{i};
        data = saved_data.(field_name)(:); 
        file_path = fullfile(folder_name, [field_name, '.csv']);

        if strcmp(field_name, 'time_all')
            T = table(time_vector, 'VariableNames', {'Time'});
        else
            T = table(time_vector, data, 'VariableNames', {'Time', field_name});
        end
        writetable(T, file_path);
    end
    disp('All saved data with time have been exported to CSV files.');

end

% ===========================================================================
% ==== FUNCTION FOR INITIAL MISSILE GEOMETRIC PROPERTIES ====
% ===========================================================================

function [mass, fuel_mass, xcg, Iyy] = compute_missile_geometric_properties()
    in2m = 0.0254;
    lb_in3_to_kg_m3 = 27679.9;
    lb_to_kg = 0.453592;

    % === Geometry in meters ===
    diameter = 12 * in2m;
    r = diameter / 2;

    % Nose
    nose_L = 36 * in2m;
    nose_rho = 0.1 * lb_in3_to_kg_m3;
    nose_xcg = (2/3) * nose_L;
    nose_volume = (1/3) * pi * r^2 * nose_L;
    nose_mass = nose_volume * nose_rho;
    Iyy_nose = (3/20) * nose_mass * r^2;

    % GNC
    GNC_L = 36 * in2m;
    GNC_rho = 0.1 * lb_in3_to_kg_m3;
    GNC_xcg = GNC_L/2 + nose_xcg;
    GNC_volume = pi * r^2 * GNC_L;
    GNC_mass = GNC_volume * GNC_rho;
    Iyy_GNC = (1/12) * GNC_mass * (3*r^2 + GNC_L^2);

    % Warhead
    warhead_L = 72 * in2m;
    warhead_rho = 0.13 * lb_in3_to_kg_m3;
    warhead_xcg = warhead_L/2 + GNC_xcg;
    warhead_volume = pi * r^2 * warhead_L;
    warhead_mass = warhead_volume * warhead_rho;
    Iyy_warhead = (1/12) * warhead_mass * (3*r^2 + warhead_L^2);

    % Fuel
    fuel_L = 96 * in2m;
    full_fuel_rho = 0.11 * lb_in3_to_kg_m3;
    empty_fuel_rho = 0.06 * lb_in3_to_kg_m3;
    full_fuel_xcg = fuel_L/2 + warhead_xcg;
    fuel_volume = pi * r^2 * fuel_L;
    full_fuel_mass = fuel_volume * full_fuel_rho;
    empty_fuel_mass = fuel_volume * empty_fuel_rho;
    Iyy_fuel = (1/12) * full_fuel_mass * (3*r^2 + fuel_L^2);

    % Fins
    rho_fin = 0.12 * lb_in3_to_kg_m3;
    fin_thickness = 0.2 * in2m;

    front_chord = 72 * in2m;
    front_span = 24 * in2m;
    front_root = 84 * in2m;
    front_area = 0.5 * front_chord * front_span;
    front_volume = front_area * fin_thickness;
    front_mass = front_volume * rho_fin * 4;
    front_xcg = front_root + front_chord / 3;
    Iyy_front_fins = 4 * (1/36) * (front_mass / 4) * (front_chord^2 + front_span^2);

    rear_chord = 24 * in2m;
    rear_span = 24 * in2m;
    rear_root = 216 * in2m;
    rear_area = 0.5 * rear_chord * rear_span;
    rear_volume = rear_area * fin_thickness;
    rear_mass = rear_volume * rho_fin * 4;
    rear_xcg = rear_root + rear_chord / 3;
    Iyy_rear_fins = 4 * (1/36) * (rear_mass / 4) * (rear_chord^2 + rear_span^2);

    % Mass and fuel
    mass = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + full_fuel_mass;
    fuel_mass = full_fuel_mass - empty_fuel_mass;

    % xcg
    xcg = (nose_mass * nose_xcg + GNC_mass * GNC_xcg + warhead_mass * warhead_xcg + ...
           full_fuel_mass * full_fuel_xcg + front_mass * front_xcg + rear_mass * rear_xcg) / mass;

    % Distances
    d_nose = abs(nose_xcg - xcg);
    d_GNC = abs(GNC_xcg - xcg);
    d_warhead = abs(warhead_xcg - xcg);
    d_fuel = abs(full_fuel_xcg - xcg);
    d_front = abs(front_xcg - xcg);
    d_rear = abs(rear_xcg - xcg);

    Iyy_total = ...
        Iyy_nose + nose_mass * d_nose^2 + ...
        Iyy_GNC + GNC_mass * d_GNC^2 + ...
        Iyy_warhead + warhead_mass * d_warhead^2 + ...
        Iyy_fuel + full_fuel_mass * d_fuel^2 + ...
        Iyy_front_fins + front_mass * d_front^2 + ...
        Iyy_rear_fins + rear_mass * d_rear^2;

    Iyy = Iyy_total; 
end


% ===========================================================================
% ==== FUNCTION FOR UPDATING MISSILE GEOMETRIC PROPERTIES AFTER THRUST====
% ===========================================================================

function [xcg, Iyy] = update_xcg_Iyy(current_fuel_mass)
    in2m = 0.0254;
    lb_in3_to_kg_m3 = 27679.9;

    diameter = 12 * in2m;
    r = diameter / 2;

    % Nose
    nose_L = 36 * in2m;
    nose_rho = 0.1 * lb_in3_to_kg_m3;
    nose_xcg = (2/3) * nose_L;
    nose_volume = (1/3) * pi * r^2 * nose_L;
    nose_mass = nose_volume * nose_rho;
    Iyy_nose = (3/20) * nose_mass * r^2;

    % GNC
    GNC_L = 36 * in2m;
    GNC_rho = 0.1 * lb_in3_to_kg_m3;
    GNC_xcg = GNC_L/2 + nose_xcg;
    GNC_volume = pi * r^2 * GNC_L;
    GNC_mass = GNC_volume * GNC_rho;
    Iyy_GNC = (1/12) * GNC_mass * (3*r^2 + GNC_L^2);

    % Warhead
    warhead_L = 72 * in2m;
    warhead_rho = 0.13 * lb_in3_to_kg_m3;
    warhead_xcg = warhead_L/2 + GNC_xcg;
    warhead_volume = pi * r^2 * warhead_L;
    warhead_mass = warhead_volume * warhead_rho;
    Iyy_warhead = (1/12) * warhead_mass * (3*r^2 + warhead_L^2);

    % Fuel
    fuel_L = 96 * in2m;
    structure_rho = 0.06 * lb_in3_to_kg_m3;
    fuel_volume = pi * r^2 * fuel_L;
    fuel_structure_mass = fuel_volume * structure_rho;
    fuel_mass = current_fuel_mass + fuel_structure_mass;
    fuel_xcg = fuel_L/2 + warhead_xcg;
    Iyy_fuel = (1/12) * fuel_mass * (3*r^2 + fuel_L^2);

    % Fins
    rho_fin = 0.12 * lb_in3_to_kg_m3;
    fin_thickness = 0.2 * in2m;

    front_chord = 72 * in2m;
    front_span = 24 * in2m;
    front_root = 84 * in2m;
    front_area = 0.5 * front_chord * front_span;
    front_volume = front_area * fin_thickness;
    front_mass = front_volume * rho_fin * 4;
    front_xcg = front_root + front_chord / 3;
    Iyy_front_fins = 4 * (1/36) * (front_mass / 4) * (front_chord^2 + front_span^2);

    rear_chord = 24 * in2m;
    rear_span = 24 * in2m;
    rear_root = 216 * in2m;
    rear_area = 0.5 * rear_chord * rear_span;
    rear_volume = rear_area * fin_thickness;
    rear_mass = rear_volume * rho_fin * 4;
    rear_xcg = rear_root + rear_chord / 3;
    Iyy_rear_fins = 4 * (1/36) * (rear_mass / 4) * (rear_chord^2 + rear_span^2);

    % Mass
    mass = nose_mass + GNC_mass + warhead_mass + front_mass + rear_mass + fuel_mass;

    % xcg
    xcg = (nose_mass * nose_xcg + GNC_mass * GNC_xcg + warhead_mass * warhead_xcg + ...
           fuel_mass * fuel_xcg + front_mass * front_xcg + rear_mass * rear_xcg) / mass;

    % Distances
    d_nose = abs(nose_xcg - xcg);
    d_GNC = abs(GNC_xcg - xcg);
    d_warhead = abs(warhead_xcg - xcg);
    d_fuel = abs(fuel_xcg - xcg);
    d_front = abs(front_xcg - xcg);
    d_rear = abs(rear_xcg - xcg);

    Iyy = ...
        Iyy_nose + nose_mass * d_nose^2 + ...
        Iyy_GNC + GNC_mass * d_GNC^2 + ...
        Iyy_warhead + warhead_mass * d_warhead^2 + ...
        Iyy_fuel + fuel_mass * d_fuel^2 + ...
        Iyy_front_fins + front_mass * d_front^2 + ...
        Iyy_rear_fins + rear_mass * d_rear^2;
end



