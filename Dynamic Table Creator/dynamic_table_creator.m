tic;
profile off

clear
close all
clc
clearvars

global forces_moment aero_prop missile;
missile = struct();
constants = struct();
target = struct();  
thrust = struct();
sim = struct();         
coef_data = struct();
sim_data = struct();

% Useful Constants
%=============================================================================================
constants.d2r             = pi/180;
constants.r2d             = 180/pi;
constants.m2ft            = 3.28084;
constants.kg2slug         = 0.0685218;
constants.kg2lb           = 2.20462;
constants.kgm2_to_lbft2   = 23.73036;
constants.ft2m            = 0.3048;
constants.slug2kg         = 14.5939;
constants.lb2kg           = 0.453592;
constants.lbft2_to_kgm2   = 0.0421401;
constants.g               = 9.81;
%=============================================================================================

% User Inputs:
% Target
R_Tx = 8000;                          % Target Position X-Component (m)
R_Tz = 8000;                          % Target Position Y-Component (m)
target.V_T = 300;                     % Velocity of the target (m/s)
Beta = -15;                           % Angle of the target velocity (degree)   
target.XNT = -2*constants.g;          % Target maneuver acceleration [m/s²]
 
% Case II Initial Missile
R_Mx = 0;                             % Missile Position X-Component (m)
R_Mz = 1000;                             % Missile Position Y-Component (m) 
V_M = 500;                            % Initial missile velocity (m/s)
q = 0;
HE = -5;
missile.XNP = 3;                      % Effective navigation ratio

% Missile Paramaters
missile.lref = 0.3048;                % missile diameter [m]
missile.S = pi * missile.lref^2 / 4;  % reference area [m^2]
% [missile.mass, missile.fuel_mass, missile.xcg, missile.Iyy] = compute_missile_geometric_properties();
missile.mass = 1072.1;
missile.xcg = 2.1511;
missile.Iyy = 1175.9;

%=============================================================================================
% Initial Calculations:

% Convert angles to radians
Beta = Beta * pi/180;
HE_rad = HE * pi/180;
% Length of the Line of sight and Components (m)
R_TMx = R_Tx - R_Mx;                      % Length of the Line of sight X-Component (m)
R_TMz = R_Tz - R_Mz;                      % Length of the Line of sight Z-Component (m)
R_TM = sqrt(R_TMx*R_TMx + R_TMz*R_TMz);   % Length of the Line of sight (m)

Lamda = atan2(R_TMz , R_TMx);             % Line-of-sight angle (rad)
Lead = asin(target.V_T*sin(Beta+Lamda)/V_M);     % Missile lead angle

% Target Velocity Components (m/s)
V_Tx = -target.V_T * cos(Beta);                  % Target velocity X-Component (m/s)
V_Tz =  target.V_T * sin(Beta);                  % Target velocity Y-Component (m/s)
theta = Lamda + Lead + HE_rad;

% Missile Velocity Components (m/s)
V_Mx = V_M * cos(theta);
V_Mz = V_M * sin(theta);

% Velocity of the Line of sight Components (m/s)
V_TMx = V_Tx - V_Mx;                      % Velocity of the Line of sight X-Component (m/s)
V_TMz = V_Tz - V_Mz;                      % Velocity of the Line of sight Y-Component (m/s)
V_C = -(R_TMx * V_TMx + R_TMz * V_TMz) / R_TM;                     % Closing velocity (m/s)


% % Extract time and signal values into a struct
% simData.T        = simOut.tout;
% simData.R_Tx     = simOut.ScopeData_RTx.signals.values;
% simData.R_Tz     = simOut.ScopeData_RTz.signals.values;
% simData.V_Tx     = simOut.ScopeData_VTx.signals.values;
% simData.V_Ty     = simOut.ScopeData_VTz.signals.values;
% simData.R_Mx     = simOut.ScopeData_RMx.signals.values; 
% simData.R_Mz     = simOut.ScopeData_RMz.signals.values;
% simData.V_Mx     = simOut.ScopeData_VMx.signals.values; 
% simData.V_Mz     = simOut.ScopeData_VMz.signals.values;
% simData.A_Mx     = simOut.ScopeData_AMx.signals.values;
% simData.A_Mz     = simOut.ScopeData_AMz.signals.values;
% 
% simData.V_C      = simOut.ScopeData_VC.signals.values;
% simData.XNCG     = simOut.ScopeData_XNCG.signals.values;
% simData.R_TM     = simOut.ScopeData_RTM.signals.values;
% 
% simData.Beta_Rad = simOut.ScopeData_BetaAngle.signals.values;
% simData.Lamda    = simOut.ScopeData_Lamda.signals.values;
% simData.Lamda_D  = simOut.ScopeData_LamdaD.signals.values;
% 
% simData.CN       = simOut.ScopeData_CN.signals.values;
% simData.CNA      = simOut.ScopeData_CNA.signals.values;
% simData.CND      = simOut.ScopeData_CND.signals.values;
% simData.CM       = simOut.ScopeData_CM.signals.values;
% simData.CMA      = simOut.ScopeData_CMA.signals.values;
% simData.CMD      = simOut.ScopeData_CMD.signals.values;
% simData.CMQ      = simOut.ScopeData_CMQ.signals.values;
% simData.CA       = simOut.ScopeData_CA.signals.values;
% 
% simData.Rho      = simOut.ScopeData_Rho.signals.values; 
% simData.Va       = simOut.ScopeData_Va.signals.values;  
% simData.Theta    = simOut.ScopeData_Theta.signals.values;
% simData.ThetaD   = simOut.ScopeData_ThetaD.signals.values;
% simData.ThetaDD  = simOut.ScopeData_ThetaDD.signals.values;
% 
% simData.V        = simOut.ScopeData_V.signals.values; 
% simData.Q        = simOut.ScopeData_Q.signals.values; 
% simData.Lf       = simOut.ScopeData_Lf.signals.values;
% simData.Df       = simOut.ScopeData_Df.signals.values;
% simData.Xf       = simOut.ScopeData_Xf.signals.values;
% simData.Zf       = simOut.ScopeData_Zf.signals.values;
% simData.M        = simOut.ScopeData_M.signals.values;
% simData.MACH     = simOut.ScopeData_MACH.signals.values; 
% simData.Alpha    = simOut.ScopeData_Alpha.signals.values; 
% simData.Iyy      = simOut.ScopeData_Iyy.signals.values; 
% simData.XCG      = simOut.ScopeData_XCG.signals.values; 
% simData.mass     = simOut.ScopeData_mass.signals.values; 

%-----------------------------------------


% Initialize Coefficients
[XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, AERO_grid] = read_coef();
[alt_table, V_a_table, rho_table, V_air_interp, rho_interp] = read_air_data();


% Simulation settings
sim.t = 0.;                                  % Initial time (s)
sim.n = 0.;                                  % Length of the saved array
sim.h = 0;                                   % Integration step size (s)
sim.flag = false;
X_s = [R_Tx; R_Tz; V_Tx; V_Tz; Beta; R_Mx; R_Mz; V_Mx; V_Mz; theta; q];

L = length(X_s);
dynamic_data_log = [];
dot_graph_log = [];
dot_graph_index = 0;
% MAIN LOOP
% Integration using "Runge Kutta 4th Order Method"     
while V_C >= 0  && R_TM >= 0
    if R_TM < 500 
        sim.h = 0.001;                        % Integration step size (s)
    else
        sim.h = 0.001;
    end

  
    sim.t = sim.t + sim.h;                    % Increase time with step size  
    sim.n = sim.n + 1;                        % 
  
    % SIM BEGIN
    X_s = RK4(X_s, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp);
    % SIM END
% if sim.t >= 2.249
%     keyboard;  
% end   
    % Extract updated state variables
    R_Tx    = X_s(1);
    R_Tz    = X_s(2);
    Beta    = X_s(5);
    R_Mx    = X_s(6);
    R_Mz    = X_s(7);
    V_Mx    = X_s(8);
    V_Mz    = X_s(9);
    theta   = X_s(10);
    q       = X_s(11);


    % Recompute alpha and velocity for current state
    [u_b, w_b] = transform(V_Mx, V_Mz, theta, "I2B");
    alpha = atan2(w_b, u_b)*180/pi;

    X_s(3) = -target.V_T*cos(Beta);                                 % Target velocity X-Component (m/s) 
    X_s(4) =  target.V_T*sin(Beta);                                 % Target velocity Y-Component (m/s) 
    V_Tx = X_s(3);
    V_Tz = X_s(4); 
    
    % Recompute Line-of-Sight (LOS) variables
    R_TMx = R_Tx - R_Mx;                                            % LOS X-component
    R_TMz = R_Tz - R_Mz;                                            % LOS Z-component
    R_TM = sqrt(R_TMx^2 + R_TMz^2);                                 % LOS magnitude
    V_TMx = V_Tx - V_Mx;                                            % Relative velocity X
    V_TMz = V_Tz - V_Mz;                                            % Relative velocity Z
    Lamda = atan2(R_TMz, R_TMx);                                    % LOS angle
    Lamda_d = (V_TMz * R_TMx - V_TMx * R_TMz) / (R_TM^2);           % LOS rate
    V_C = -(R_TMx * V_TMx + R_TMz * V_TMz) / R_TM;                  % Closing velocity
    XNC = missile.XNP * V_C * Lamda_d;                              % Desired acceleration command (m/s²)

    if mod(sim.n, 1000) == 0    
    dynamic_data.t       = sim.t;
    dynamic_data.alpha   = alpha;
    dynamic_data.Q       = aero_prop.Q;
    dynamic_data.rho     = aero_prop.rho;
    dynamic_data.h       = X_s(7);
    dynamic_data.V       = sqrt(X_s(8)^2 + X_s(9)^2);
    dynamic_data.MACH    = missile.MACH;
    dynamic_data.mass    = missile.mass;
    dynamic_data.Iyy     = missile.Iyy;
    dynamic_data.xcg     = missile.xcg;
    dynamic_data.CMA     = aero_prop.aero_derivatives(6);
    dynamic_data.CNA     = aero_prop.aero_derivatives(7);
    dynamic_data.CMD     = aero_prop.aero_derivatives(11)/0.0174533;
    dynamic_data.CND     = aero_prop.aero_derivatives(10)/0.0174533;
    dynamic_data.CMQ     = aero_prop.aero_derivatives(8);
    
    dot_graph.Mx = R_Mx;
    dot_graph.Mz = R_Mz;
    dot_graph_index = dot_graph_index + 1;
    dot_graph.index = dot_graph_index;

    % Append dynamic_data as a row to dynamic_data_log
    dynamic_data_log = [dynamic_data_log; ...
        dynamic_data.t, dynamic_data.alpha, dynamic_data.Q, dynamic_data.rho, ...
        dynamic_data.h, dynamic_data.V, dynamic_data.MACH, ...
        dynamic_data.mass, dynamic_data.Iyy, dynamic_data.xcg, ...
        dynamic_data.CMA, dynamic_data.CNA, dynamic_data.CMD, dynamic_data.CND, dynamic_data.CMQ];
    dot_graph_log = [dot_graph_log; dot_graph.Mx, dot_graph.Mz, dot_graph.index];
    end 
    % if mod(sim.n, 500) == 0 || R_TM < 0.5 || V_C < 1e-3    
    % fprintf('Step %d, Time %.4f sec\n', sim.n, sim.t);
    % fprintf('alpha  = %.4f deg\n', alpha);
    % fprintf('x      = %.4f m\n', X_s(6));
    % fprintf('z      = %.4f m\n', X_s(7));
    % fprintf('u      = %.4f m/s\n', X_s(8));
    % fprintf('w      = %.4f m/s\n', X_s(9));
    % fprintf('theta  = %.4f deg\n', X_s(10)*180/pi);
    % fprintf('q      = %.4f deg/s\n', X_s(11)*180/pi);
    % fprintf('----------------------------------\n');   
    % end

    % Save data
    save_data(sim, X_s, missile, forces_moment, aero_prop, R_TMx, R_TMz, V_TMx, V_TMz, Lamda, Lamda_d, V_C, XNC,alpha) ;

end 

% Column headers for CSV
dynamic_data_header = {'t', 'alpha', 'Q','rho', 'h', 'V', 'MACH', ...
          'mass', 'Iyy', 'xcg', 'CMA', 'CNA', 'CMD', 'CND', 'CMQ'};
dot_graph_header = {'Mx', 'Mz', 'index'};

% Convert matrix to table
dynamic_data_table = array2table(dynamic_data_log, 'VariableNames', dynamic_data_header);
dot_graph_table = array2table(dot_graph_log, 'VariableNames', dot_graph_header);

% Write to CSV file
writetable(dynamic_data_table, 'dynamic_data.csv');
fprintf('dynamic_data.csv has been saved successfully.\n');

fig = figure(); 
hold on 
plot(dot_graph_table.Mx, dot_graph_table.Mz, LineStyle="none", Marker='.', MarkerSize=40)
ylabel('Z(m)');
xlabel('X(m)');
fontsize(14,"points");
text(dot_graph_table.Mx, dot_graph_table.Mz, cellfun(@(c) sprintf("    %i", c),num2cell(dot_graph_table.index)), FontSize=14);
hold off



fprintf('Missile hit the target at %.2f s\n', sim.t);
sim.flag = true;

%============================================================================================= 

saved_data = save_data(sim, X_s, missile, forces_moment, aero_prop, R_TMx, R_TMz, V_TMx, V_TMz, Lamda, Lamda_d, V_C, XNC,alpha);

% plot_data(saved_data, simData);
export_saved_data_to_csv(saved_data);

elapsed_time = toc;  % Stop measuring time and get the elapsed time
fprintf('\n Simulation completed in %.3f seconds.\n', elapsed_time);
% profile viewer



% ==== DYNAMICS FUNCTION ====
function dX = missile_dynamics(X, sim, constants, thrust, target, AERO_grid, V_air_interp, rho_interp)
    global forces_moment aero_prop missile;
    
    R_Tx    = X(1);
    R_Tz    = X(2);
    Beta    = X(5);
    V_Tx    = -target.V_T*cos(Beta);                          
    V_Tz    =  target.V_T*sin(Beta);          
    R_Mx    = X(6);
    R_Mz    = X(7);
    V_Mx    = X(8);
    V_Mz    = X(9);
    theta   = X(10);
    theta_d = X(11);
 

    R_TMx = R_Tx - R_Mx;                                    % Length of the Line of sight X-Component (m) 
    R_TMz = R_Tz - R_Mz;                                    % Length of the Line of sight Y-Component (m) 
    R_TM = sqrt(R_TMx*R_TMx + R_TMz*R_TMz);                 % Length of the Line of sight (m) 
    V_TMx = V_Tx - V_Mx;                                    % Velocity of the Line of sight X-Component (m/s) 
    V_TMz = V_Tz - V_Mz;                                    % Velocity of the Line of sight Y-Component (m/s) 
    V_C = -(R_TMx*V_TMx + R_TMz*V_TMz)/R_TM;                % Closing velocity (m/s) 
    Lamda = atan2(R_TMz,R_TMx);                             % Line-of-sight angle (rad) 
    Lamda_d = (V_TMz*R_TMx - V_TMx*R_TMz)/(R_TM*R_TM);      % Time derivative of line-of-sight angle
    XNC = missile.XNP*V_C*Lamda_d;                          % Desired acceleration command (m/s^2) 
    A_Mx = -XNC*sin(Lamda);                                 % Missile acceleration X-Component (m/s^2)
    A_Mz =  XNC*cos(Lamda);                                 % Missile acceleration Y-Component (m/s^2)
    Beta_d = target.XNT/target.V_T;                         % Time derivative of the Beta angle

% if sim.t >= 2.249
%     keyboard;  
% end   

    % Calculate aero forces    
    [u_B, w_B] = transform(V_Mx, V_Mz, theta, "I2B");
    alpha = atan2(w_B, u_B);
    V = sqrt(u_B^2 + w_B^2);
    delta = 10;
    [aero_derivatives, rho, MACH] = get_derivatives(missile.xcg, alpha * 180 / pi, V, R_Mz, delta, AERO_grid, V_air_interp, rho_interp);
    CN = aero_derivatives(1);  
    CA = aero_derivatives(3); 

    Q = 0.5*rho*V^2;

    % Q_all(sim.n) = aero_prop.Q;                        % Dynamic Pressure
    % CN_all(sim.n) = aero_prop.aero_derivatives(1);     % Longitudinal Force Coefficient
    % CM_all(sim.n) = aero_prop.aero_derivatives(2);     % Pitching Moment Coefficient
    % CA_all(sim.n) = aero_prop.aero_derivatives(3);     % Axial Force Coefficient
    % CL_all(sim.n) = aero_prop.aero_derivatives(4);     % Lift Coefficient
    % CD_all(sim.n) = aero_prop.aero_derivatives(5);     % Drag Coefficient
    % CMA_all(sim.n) = aero_prop.aero_derivatives(6);    % Derivative of Pitching Moment Coefficient with respect to Angle of Attack
    % CNA_all(sim.n) = aero_prop.aero_derivatives(7);    
    % CMQ_all(sim.n) = aero_prop.aero_derivatives(8);    % Derivative of Pitching Moment Coefficient with respect to pitch rate
    % CNQ_all(sim.n) = aero_prop.aero_derivatives(9);
    % CND_all(sim.n) = aero_prop.aero_derivatives(10);
    % CMD_all(sim.n) = aero_prop.aero_derivatives(11);
    
    [Lf, Df, X_force, Z_force] = calculate_aero_forces(CN, CA, Q, missile.S);

    ud_B = (X_force) / missile.mass - constants.g*sin(theta);
    wd_B = Z_force / missile.mass - constants.g*cos(theta);
    [ud_In, wd_In] = transform(ud_B, wd_B, theta, "B2I");

    ud_I = ud_In + A_Mx;
    wd_I = wd_In + A_Mz;

    CM  = aero_derivatives(2);
    M = Q * missile.S * missile.lref* CM;

    theta_dd = -M / missile.Iyy;

    % Forces and moment
    forces_moment = struct();         
    forces_moment.Lf = Lf;    
    forces_moment.Df = Df;
    forces_moment.X_force = X_force;
    forces_moment.Z_force = Z_force;   
    forces_moment.M = M;

    % Aero Properties
    aero_prop = struct();         
    aero_prop.Q = Q;    
    aero_prop.rho = rho;
    aero_prop.aero_derivatives = aero_derivatives;  
    
    missile.MACH = MACH;

    dX = [V_Tx; V_Tz; 0; 0; Beta_d; V_Mx; V_Mz; ud_I; wd_I; theta_d; theta_dd;];
    % === Derivatives ===
    % dX = zeros(11,1);
    % dX(1)  = V_Tx;                         
    % dX(2)  = V_Tz;                         
    % dX(3)  = 0;                           
    % dX(4)  = 0;                            
    % dX(5)  = Beta_d;                       
    % dX(6)  = V_Mx;                         
    % dX(7)  = V_Mz;                         
    % dX(8)  = ud_I;                         
    % dX(9)  = wd_I;                         
    % dX(10) = theta_d;                     
    % dX(11) = theta_dd;                  
end


% ==== Integration ====
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

%==== Calculate aero forces ====
function[Lf, Df, X_force, Z_force] = calculate_aero_forces(CN, CA, Q, S)

    % Compute aerodynamic forces 
    Df = Q * S * CA;
    Lf = Q * S * CN;

    % Force components in body frame
    X_force = -Df;
    Z_force = -Lf;
end

% ==== Transformation ====
function [u_t, w_t] = transform(u, w, theta, mode)

    if mode == "I2B"
        u_t =  cos(theta) * u + sin(theta) * w;
        w_t = -sin(theta) * u + cos(theta) * w;
    elseif mode == "B2I"
        u_t =  cos(theta) * u - sin(theta) * w;
        w_t =  sin(theta) * u + cos(theta) * w;
    else
        error("Mode must be 'I2B' or 'B2I'");
    end
end


% ==== Plot  ====
function plot_data(saved_data, simData)

    % Target and Missile Path
    figure;
    hold on;
    
    % Original saved data
    plot(saved_data.x_t_all, saved_data.z_t_all, 'r--', 'LineWidth', 2);  % Target path
    plot(saved_data.x_all, saved_data.z_all, 'b-', 'LineWidth', 2);      % Missile path
    
    % Simulink data
    plot(simData.R_Tx, simData.R_Tz, 'm--', 'LineWidth', 1.5);  % Simulink Target path
    plot(simData.R_Mx, simData.R_Mz, 'c-', 'LineWidth', 1.5);   % Simulink Missile path
    
    xlabel('X Position (m)');
    ylabel('Z Position (m)');
    title('Target and Missile Path Comparison');
    legend('Target Path (MATLAB)', 'Missile Path (MATLAB)', ...
           'Target Path (Simulink)', 'Missile Path (Simulink)', ...
           'Location', 'best');
    grid on;

    
    % Angle of Attack
    figure;
    plot(saved_data.time_all, saved_data.alpha_all, 'b'); 
    hold on;
    plot(simData.T, simData.Alpha, 'r--');  
    xlabel('Time (s)'); ylabel('Alpha (deg)');
    title('Angle of Attack Over Time');
    legend('Alpha (MATLAB)', 'Alpha (Simulink)');
    grid on;

    % Velocity and Components
    figure;
    % u Component
    subplot(3,1,1);
    plot(saved_data.time_all, saved_data.u_all, 'r');
    hold on;
    plot(simData.T, simData.V_Mx, 'k--'); 
    ylabel('u (m/s)');
    title('Velocity Component u Over Time');
    legend('u (MATLAB)', 'u (Simulink)');
    grid on;

    % w Component
    subplot(3,1,2);
    plot(saved_data.time_all, saved_data.w_all, 'b');
    hold on;
    plot(simData.T, simData.V_Mz, 'r--');  
    ylabel('w (m/s)');
    title('Velocity Component w Over Time');
    legend('w (MATLAB)', 'w (Simulink)');
    grid on;

    % Total Velocity V
    subplot(3,1,3);
    plot(saved_data.time_all, saved_data.V_all, 'k');
    hold on;
    plot(simData.T, simData.V, 'm--');  % Simulink total velocity
    ylabel('V (m/s)');
    xlabel('Time (s)');
    title('Total Velocity Over Time');
    legend('V (MATLAB)', 'V (Simulink)');
    grid on;


    % Pitch Angle and Pitch Rate
    figure;
    % Pitch Angle
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.theta_all * 180/pi, 'b', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.Theta * 180/pi, 'r--');
    ylabel('\theta (deg)');
    title('Pitch Angle Over Time');
    legend('\theta (MATLAB)', '\theta (Simulink)');
    grid on;
    % Pitch Rate
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.thetadot_all * 180/pi, 'r', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.ThetaD * 180/pi, 'b--');  % Simulink theta dot in deg/s
    ylabel('q (deg/s)');
    xlabel('Time (s)');
    title('Pitch Rate Over Time');
    legend('q (MATLAB)', 'q (Simulink)');
    grid on;

    % Closing Velocity and Commanded Acceleration
    figure;
    % Closing Velocity
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.V_C_all, 'm', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.V_C, 'k--');
    ylabel('V_C (m/s)');
    title('Closing Velocity Over Time');
    legend('V_C (MATLAB)', 'V_C (Simulink)');
    grid on;
    % Commanded Acceleration
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.XNC_all, 'k', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.XNCG, 'r--');
    ylabel('XNC (g)'); xlabel('Time (s)');
    title('Commanded Acceleration Over Time');
    legend('XNC (MATLAB)', 'XNC (Simulink)');
    grid on;

    %  Line-of-Sight Angle and Rate 
    figure;
    % Lambda
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.Lamda_all * 180/pi, 'b', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.Lamda * 180/pi, 'r--');
    ylabel('\lambda (deg)');
    title('Line-of-Sight Angle Over Time');
    legend('\lambda (MATLAB)', '\lambda (Simulink)');
    grid on;
    % Lambda_dot
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.Lamda_d_all * 180/pi, 'r', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.Lamda_D * 180/pi, 'b--');
    ylabel('d\lambda/dt (deg/s)');
    xlabel('Time (s)');
    title('LOS Angle Rate Over Time');
    legend('LamdaD (MATLAB)', 'LamdaD (Simulink)');
    grid on;


    % Air Density and Dynamic Pressure
    figure;
    % Air Density
    subplot(2,1,1);
    plot(saved_data.time_all, saved_data.rho_all, 'b', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.Rho, 'r--');
    ylabel('Density (kg/m³)');
    title('Air Density Over Time');
    legend('\rho (MATLAB)', '\rho (Simulink)');
    grid on;

    % Dynamic Pressure
    subplot(2,1,2);
    plot(saved_data.time_all, saved_data.Q_all, 'r', 'LineWidth', 1.5);
    hold on;
    plot(simData.T, simData.Q, 'b--');  % Simulink dynamic pressure
    ylabel('Q (Pa)');
    xlabel('Time (s)');
    title('Dynamic Pressure Over Time');
    legend('Q (MATLAB)', 'Q (Simulink)');
    grid on;

    % Forces
    figure;
    % Lift Force
    subplot(2,2,1);
    plot(saved_data.time_all, saved_data.Lf_all, 'r');
    hold on;
    plot(simData.T, simData.Lf, 'b--');
    ylabel('Lift Force');
    title('Lift Force Over Time');
    legend('Lf (MATLAB)', 'Lf (Simulink)');
    grid on;
    % Z Force
    subplot(2,2,3);
    plot(saved_data.time_all, saved_data.Z_force_all, 'b');
    hold on;
    plot(simData.T, simData.Zf, 'r--');
    ylabel('Z Force');
    xlabel('Time (s)');
    title('Z Force Over Time');
    legend('Zf (MATLAB)', 'Zf (Simulink)');
    grid on;
    % Drag Force
    subplot(2,2,2);
    plot(saved_data.time_all, saved_data.Df_all, 'r');
    hold on;
    plot(simData.T, simData.Df, 'b--');
    ylabel('Drag Force');
    title('Drag Force Over Time');
    legend('Df (MATLAB)', 'Df (Simulink)');
    grid on;
    % X Force
    subplot(2,2,4);
    plot(saved_data.time_all, saved_data.X_force_all, 'b');
    hold on;
    plot(simData.T, simData.Xf, 'r--');
    ylabel('X Force');
    xlabel('Time (s)');
    title('X Force Over Time');
    legend('Xf (MATLAB)', 'Xf (Simulink)');
    grid on;

    % Moment
    figure;
    plot(saved_data.time_all, saved_data.M_all, 'b'); 
    hold on;
    plot(simData.T, simData.M, 'r--');
    xlabel('Time (s)'); ylabel('Moment');
    title('Moment Over Time');
    legend('M (MATLAB)', 'M (Simulink)');
    grid on;

    %  Aerodynamic Coefficients 
    figure;
    % CM
    subplot(3,2,1);
    plot(saved_data.time_all, saved_data.CM_all, 'b'); 
    hold on;
    plot(simData.T, simData.CM, 'r--');  % Simulink CM
    xlabel('Time (s)'); ylabel('CM');
    title('CM Over Time');
    legend('CM (MATLAB)', 'CM (Simulink)');
    grid on;
    % CMA
    subplot(3,2,3);
    plot(saved_data.time_all, saved_data.CMA_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CMA');
    title('CMA Over Time');
    % CMQ
    subplot(3,2,5);
    plot(saved_data.time_all, saved_data.CMQ_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CMQ');
    title('CMQ Over Time');
    % CL
    subplot(3,2,2);
    plot(saved_data.time_all, saved_data.CL_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CL');
    title('CL Over Time');
    % CD
    subplot(3,2,4);
    plot(saved_data.time_all, saved_data.CD_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CD');
    title('CD Over Time');
    % CA
    subplot(3,2,6);
    plot(saved_data.time_all, saved_data.CA_all, 'b'); 
    hold on;
    plot(simData.T, simData.CA, 'r--');  % Simulink CA
    xlabel('Time (s)'); ylabel('CA');
    title('CA Over Time');
    legend('CA (MATLAB)', 'CA (Simulink)');
    grid on;
end

function [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, AERO_grid] = read_coef()
    folderPath = fullfile(pwd, 'coef_output_def');
    csvFiles = dir(folderPath);
    csvFiles = csvFiles(3:end);  
    N_xcg = length(csvFiles);
    xcgFiles = strings(N_xcg, 1);
    
    for i = 1:N_xcg
        xcgFiles(i) = fullfile(csvFiles(i).folder, csvFiles(i).name);
    end

    % We assume all xcg directories contain the same alpha files
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

    % === Step 1: Read data ===
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

    % === Step 2: Sort alpha and xcg ===
    [alpha_vals, alpha_sort_idx] = sort(alpha_vals);
    [xcg_vals, xcg_sort_idx] = sort(xcg_vals);

    for f = 1:length(fields)
        for i = 1:N_xcg
            data_grid(i).(fields{f}) = data_grid(i).(fields{f})(alpha_sort_idx, :, :);
        end
    end

    % === Step 3: Create ndgrid ===
    [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid] = ndgrid(xcg_vals, alpha_vals, mach_vals, delta_vals);

    % === Step 4: Compute CND and CMD using central difference ===
    CND_grid = nan(N_xcg, N_alpha, N_mach, N_delta);
    CMD_grid = nan(N_xcg, N_alpha, N_mach, N_delta);

    for i = 1:N_xcg
        CN = squeeze(data_grid(i).CN); % [N_alpha x N_mach x N_delta]
        CM = squeeze(data_grid(i).CM);

        for k = 1:N_alpha
            for m = 1:N_mach
                for d = 1:N_delta
                    delta = delta_vals(d);

                    % Skip endpoints (no central difference possible)
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

    % === Step 5: Build interpolants ===
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




% ==== Parse Air Velocity for MATLAB====
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

% ==== Parse Aerodynamic Data for Simulink====
function [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, AERO_grid] = read_coef_sim()
    folderPath = fullfile(pwd, 'coef_output_def');
    xcgDirs = dir(folderPath);
    xcgDirs = xcgDirs([xcgDirs.isdir] & ~startsWith({xcgDirs.name}, '.'));  % exclude . and ..

    N_xcg = length(xcgDirs);
    xcgFiles = strings(N_xcg, 1);

    for i = 1:N_xcg
        xcgFiles(i) = fullfile(folderPath, xcgDirs(i).name);
    end

    % Assume all xcg directories have same alpha files
    sampleAlphaFiles = dir(fullfile(xcgFiles(1), 'alpha*.csv'));
    N_alpha = length(sampleAlphaFiles);

    fields = {'CN','CM','CA','CL','CD','CMA','CNA','CMQ','CNQ'};

    alpha_vals = zeros(N_alpha, 1);
    xcg_vals = zeros(N_xcg, 1);
    DELTA_vals = [];

    % Read sample file to get MACH and DELTA values
    T0 = readtable(fullfile(sampleAlphaFiles(1).folder, sampleAlphaFiles(1).name));
    MACH_sample = unique(T0.MACH)';
    DELTA_vals = unique(T0.DELTA)';
    N_mach = length(MACH_sample);
    N_delta = length(DELTA_vals);

    raw_data = struct();
    for f = 1:length(fields)
        raw_data.(fields{f}) = zeros(N_xcg, N_alpha, N_mach, N_delta);
    end

    % Read all CSVs
    for i = 1:N_xcg
        alphaFiles = dir(fullfile(xcgFiles(i), 'alpha*.csv'));
        for k = 1:N_alpha
            T = readtable(fullfile(alphaFiles(k).folder, alphaFiles(k).name));

            if i == 1
                alpha_vals(k) = T.ALPHA(1);
            end
            xcg_vals(i) = T.XCG(1);

            for f = 1:length(fields)
                for m = 1:N_mach
                    for d = 1:N_delta
                        idx = T.MACH == MACH_sample(m) & T.DELTA == DELTA_vals(d);
                        val = T.(fields{f})(idx);
                        if ~isempty(val)
                            raw_data.(fields{f})(i, k, m, d) = val(1);
                        else
                            raw_data.(fields{f})(i, k, m, d) = NaN;
                        end
                    end
                end
            end
        end
    end

    % Sort coordinates
    [alpha_vals, alpha_sort_idx] = sort(alpha_vals);
    [xcg_vals, xcg_sort_idx] = sort(xcg_vals);

    for f = 1:length(fields)
        raw_data.(fields{f}) = raw_data.(fields{f})(xcg_sort_idx, alpha_sort_idx, :, :);
    end

    % Generate 4D mesh grids
    [XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid] = ndgrid(xcg_vals, alpha_vals, MACH_sample, DELTA_vals);

    % Compute central-difference CMD and CND
    CND = nan(size(raw_data.CN));
    CMD = nan(size(raw_data.CM));

    for d = 2:N_delta-1
        d_prev = DELTA_vals(d-1);
        d_next = DELTA_vals(d+1);
        delta_step = d_next - d_prev;

        CND(:,:,:,d) = (raw_data.CN(:,:,:,d+1) - raw_data.CN(:,:,:,d-1)) / delta_step;
        CMD(:,:,:,d) = (raw_data.CM(:,:,:,d+1) - raw_data.CM(:,:,:,d-1)) / delta_step;
    end

    % Fill NaN for endpoints (optional: copy closest derivative)
    CND(:,:,:,1) = CND(:,:,:,2);
    CND(:,:,:,end) = CND(:,:,:,end-1);
    CMD(:,:,:,1) = CMD(:,:,:,2);
    CMD(:,:,:,end) = CMD(:,:,:,end-1);

    % Create interpolants
    AERO_grid = struct();
    for f = 1:length(fields)
        AERO_grid.(fields{f}) = raw_data.(fields{f});
        AERO_grid.([fields{f} '_interp']) = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, raw_data.(fields{f}), 'linear', 'none');
    end

    % Add CMD and CND interpolants
    AERO_grid.CND = CND;
    AERO_grid.CMD = CMD;
    AERO_grid.CND_interp = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, CND, 'linear', 'none');
    AERO_grid.CMD_interp = griddedInterpolant(XCG_grid, ALPHA_grid, MACH_grid, DELTA_grid, CMD, 'linear', 'none');
end


% ==== Parse Air Velocity for Simulink====
function [alt_table, V_a_table, rho_table] = read_air_data_sim()

    % Read the CSV file
    folderpath = fullfile(pwd, 'air_table.csv');
    data = readtable(folderpath);

    % Extract arrays (directly from table variables)
    alt_table = data.Altitude;    
    V_a_table = data.V_a;
    rho_table = data.rho;
end

% ==== Interpolation ====
function [aero, rho_val, mach_number] = get_derivatives(xcg, alpha, V, z, delta, AERO_grid, V_air_interp, rho_interp)
    V_air = V_air_interp(z);
    rho_val = rho_interp(z);
    mach_number = V / V_air;
    
    % Get aero coefficients including CND and CMD directly from interpolants
    fields = {'CN','CM','CA','CL','CD','CMA','CNA','CMQ','CNQ','CND','CMD'};
    res = zeros(length(fields), 1); 

    for f = 1:length(fields)
        interp_func = AERO_grid.(fields{f});
        res(f) = interp_func(xcg, alpha, mach_number, 1); % change 1->delta
    end
    
    aero = res;
end


function [aero] = get_data_mach(xcg, alpha, mach, delta, AERO_grid)
    % Include all coefficients plus control derivatives
    fields = {'CN','CM','CA','CL','CD','CMA','CNA','CMQ','CNQ','CND','CMD'};
    res = zeros(length(fields), 1);

    for f = 1:length(fields)
        interp_func = AERO_grid.(fields{f});  % Directly use the interpolant
        res(f) = interp_func(xcg, alpha, mach, 1); % change 1->delta
    end

    aero = res;
end

    
% ==== Save Data  ====
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


% ==== Export Data to CSV file format  ====
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
