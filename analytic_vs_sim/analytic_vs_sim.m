clear; clc; close all;

global alt V_a ALPHA_grid MACH_grid MATRIX_grid;
global M X_force Z_force Lf Df;
global Q aero_derivatives lref rho;
global S xcg mass g Iyy step_size;
global zeta;
% Initial Conditions
x = 0;
z = 300;
V_M = 300;
theta = 0;
theta = theta * pi/180;
% u = V_M * cos(theta);
% w = V_M * sin(theta)+.001;
u = 50;
w = 0.0000001;
q = 0;
S = 0.073;
xcg =  2.151;
lref =  0.30;
mass = 1072.1;
g = 9.81*0;
Iyy = 1175.9;

% Initial state vector 
X_s = [x; z; u; w; theta; q];

% Time setup
t0 = 0;
t = t0; 
step_size = 0.0005;
L = length(X_s);
step = 0;
sim.flag = false;

% Initialize
[ALPHA_grid, MACH_grid, MATRIX_grid] = read_coef();
[alt, V_a, rho] = read_air_data();


% Main loop
while X_s(2) >= 0 && t <= 10
    
    % Integration
    X_s = RK4(X_s, step_size);

    step = step + 1;
    t = t + step_size;

    % Save data
    save_data(sim, step, step_size, X_s, M, Lf, Df, X_force, Z_force, aero_derivatives, Q);

    % % To calculate angle of attack calculate body forces
    % [u_b, w_b] = transform(X_s(3), X_s(4), X_s(5), "I2B");

    
    fprintf('Time %d\n', t);
    % fprintf('alpha  = %.4f deg\n', atan2(w_b, u_b) * 180/pi);
    % fprintf('x      = %.4f ft\n', X_s(1));
    % fprintf('z      = %.4f ft\n', X_s(2));
    % fprintf('u      = %.4f ft/s\n', X_s(3));
    % fprintf('w      = %.4f ft/s\n', X_s(4));
    % fprintf('theta  = %.4f deg\n', X_s(5)*180/pi);
    % fprintf('q      = %.4f deg/s\n', X_s(6)*180/pi);
    % fprintf('----------------------------------\n');   
end
sim.flag = true;

[saved_data] = save_data(sim, step, step_size, X_s, M, Lf, Df, X_force, Z_force, aero_derivatives, Q);

disp('Missile has hit the ground.');
% [T_list, f_list, t_mid] = calculate_all_alpha_periods(saved_data, 0, 10);
% [omega_n, zeta, omega_d, freq_Hz, T] = compute_pitch_frequency_all_steps(saved_data, Iyy, S, lref);
compare_alpha_frequencies(saved_data, Iyy, S, lref, 0, 100);
% 
[omega_n_all, zeta_all, omega_d_all, freq_Hz_all, period_all] = compute_pitch_frequency_all_steps(saved_data, Iyy, S, lref);

% Or optionally:
% omega_n = omega_n_all(1);
% 
compare_analytical_vs_simulated_alpha(saved_data, omega_n_all, omega_d_all, zeta_all);

% plot_data(saved_data);
% export_saved_data_to_csv(saved_data);


% ==== DYNAMICS FUNCTION ====
function dX = missile_dynamics(X)
    global M X_force Z_force;
    global aero_derivatives lref; 
    global mass g Iyy Q S;

    z     = X(2);
    theta = X(5);
    q     = X(6);
    [u_b, w_b] = transform(X(3), X(4), theta, "I2B");
    
    alpha = atan2(w_b, u_b);
    V = sqrt(u_b^2 + w_b^2);

    % Calculate aero forces
    calculate_aero_forces(alpha, V, z);

    ud_B = (X_force / mass) - g*sin(theta);
    wd_B = (Z_force / mass) - g*cos(theta);
    [ud_I, wd_I] = transform(ud_B, wd_B, theta, "B2I");

    CM  = aero_derivatives(2);
    CMAD = aero_derivatives(20);
    CMQ = aero_derivatives(17);
    CMA = aero_derivatives(12);

    M = Q * S * lref* CM;

    q_d = -M / Iyy;

    % === Derivatives ===
    dX = zeros(6,1);
    dX(1) = X(3);
    dX(2) = X(4);
    dX(3) = ud_I;
    dX(4) = wd_I;
    dX(5) = q;
    dX(6) = q_d;
end


% ==== Integration ====
function X_s = RK4(X_s, step_size)

    X_tmp = X_s;
    K1 = missile_dynamics(X_tmp);

    X_tmp = X_s;
    K2 = missile_dynamics(X_tmp + 0.5 * step_size * K1);

    X_tmp = X_s;
    K3 = missile_dynamics(X_tmp + 0.5 * step_size * K2);

    X_tmp = X_s;
    K4 = missile_dynamics(X_tmp + step_size * K3);

    X_s = X_s + (step_size / 6) * (K1 + 2*K2 + 2*K3 + K4);

end

%==== Calculate aero forces ====
function calculate_aero_forces(alpha, V, z)
    global Lf Df X_force Z_force;
    global aero_derivatives Q S rho_val;

    % Get aerodynamic derivatives and dynamic pressure
    [~, aero_derivatives] = get_derivatives(alpha * 180 / pi, V, z);
    % aero_derivatives:
    % aero_derivatives(1)  = CN (Longitudinal Force Coefficient)
    % aero_derivatives(2)  = CM (Pitching Moment Coefficient)
    % aero_derivatives(3)  = CA (Aerodynamic Coefficient in the axial direction)
    % aero_derivatives(4)  = CY (Side Force Coefficient)
    % aero_derivatives(5)  = CLN (Normal Force Coefficient in the lateral direction)
    % aero_derivatives(6)  = CLL (Rolling Moment Coefficient)   
    % aero_derivatives(7)  = CL (Lift Coefficient)
    % aero_derivatives(8)  = CD (Drag Coefficient)
    % aero_derivatives(9)  = CL/CD (Lift-to-Drag Ratio)
    % aero_derivatives(10) = X-C.P. (Center of Pressure along the body length)  
    % aero_derivatives(11) = CNA (Derivative of Normal Force Coefficient with respect to Angle of Attack)
    % aero_derivatives(12) = CMA (Derivative of Pitching Moment Coefficient with respect to Angle of Attack)
    % aero_derivatives(13) = CYB (Side Force Coefficient derivative with respect to sideslip)
    % aero_derivatives(14) = CLNB (Normal Force Coefficient derivative with respect to sideslip)
    % aero_derivatives(15) = CLLB (Rolling Moment Coefficient derivative with respect to sideslip)  
    % aero_derivatives(16) = CNQ (Derivative of Normal Force Coefficient with respect to pitch rate)
    % aero_derivatives(17) = CMQ (Derivative of Pitching Moment Coefficient with respect to pitch rate)
    % aero_derivatives(18) = CAQ (Derivative of Axial Force Coefficient with respect to pitch rate)
    % aero_derivatives(19) = CNAD (Derivative of Normal Force Coefficient with respect to angle of attack)
    % aero_derivatives(20) = CMAD (Derivative of Pitching Moment Coefficient with respect to angle of attack)   
    % aero_derivatives(21) = CYR (Derivative of Side Force Coefficient with respect to yaw rate)
    % aero_derivatives(22) = CLNR (Normal Force Coefficient derivative with respect to yaw rate)
    % aero_derivatives(23) = CLLR (Rolling Moment Coefficient derivative with respect to yaw rate)
    % aero_derivatives(24) = CYP (Derivative of Side Force Coefficient with respect to roll rate)
    % aero_derivatives(25) = CLNP (Normal Force Coefficient derivative with respect to roll rate)
    % aero_derivatives(26) = CLLP (Rolling Moment Coefficient derivative with respect to roll rate)

    CN = aero_derivatives(1);  
    CA = aero_derivatives(3); 

    Q = 0.5*rho_val*V^2;

    % Compute aerodynamic forces 
    Lf = Q * S * CN;
    Df = Q * S * CA;

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
function plot_data(saved_data)
    
    % Plot trajectory in x-z plane
    figure;
    plot(saved_data.x_all, saved_data.z_all, 'b', 'LineWidth', 2);
    xlabel('x (ft)'); ylabel('z (ft)');
    title('Trajectory in x-z Plane'); grid on; axis equal;

    % Plot velocity components u and w over time
    figure;
    subplot(2,1,1); plot(saved_data.time_all, saved_data.u_all, 'r');
    ylabel('u (ft/s)'); title('Velocity Component u Over Time'); grid on;
    subplot(2,1,2); plot(saved_data.time_all, saved_data.w_all, 'b');
    ylabel('w (ft/s)'); xlabel('Time (s)');
    title('Velocity Component w Over Time'); grid on;

    % Plot total velocity over time
    figure;
    plot(saved_data.time_all, saved_data.V_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('Velocity (ft/s)');
    title('Velocity Over Time');

    % Plot pitch angle over time
    figure;
    plot(saved_data.time_all, saved_data.theta_all * 180/pi, 'b'); grid on;
    xlabel('Time (s)'); ylabel('Theta (deg)');
    title('Pitch Angle Over Time');

    % Plot pitch rate over time
    figure;
    plot(saved_data.time_all, saved_data.q_all * 180/pi, 'b'); grid on;
    xlabel('Time (s)'); ylabel('q (deg/s)');
    title('Pitch Rate Over Time');

    % Plot angle of attack over time
    figure;
    plot(saved_data.time_all, saved_data.alpha_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('Alpha (deg)');
    title('Angle of Attack Over Time');

    % Plot air density over time
    figure;
    plot(saved_data.time_all, saved_data.rho_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('rho(slugs/ft³)');
    title('Air Density Over Time');

    % Plot moment over time
    figure;
    plot(saved_data.time_all, saved_data.M_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('Moment');
    title('Moment Over Time');

    % Plot lift force and Z force over time
    figure;
    subplot(2,1,1); plot(saved_data.time_all, saved_data.Lf_all, 'r');
    ylabel('Lift Force'); title('Lift Force Over Time'); grid on;
    subplot(2,1,2); plot(saved_data.time_all, saved_data.Z_force_all, 'b');
    ylabel('Z Force'); xlabel('Time (s)');
    title('Z Force Over Time'); grid on;

    % Plot drag force and X force over time
    figure;
    subplot(2,1,1); plot(saved_data.time_all, saved_data.Df_all, 'r');
    ylabel('Drag Force'); title('Drag Force Over Time'); grid on;
    subplot(2,1,2); plot(saved_data.time_all, saved_data.X_force_all, 'b');
    ylabel('X Force'); xlabel('Time (s)');
    title('X Force Over Time'); grid on;

    % Plot CM over time
    figure;
    plot(saved_data.time_all, saved_data.CM_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CM');
    title('CM Over Time');

    % Plot CL over time
    figure;
    plot(saved_data.time_all, saved_data.CL_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CL');
    title('CL Over Time');

    % Plot CD over time
    figure;
    plot(saved_data.time_all, saved_data.CD_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CD');
    title('CD Over Time');

    % Plot CMA over time
    figure;
    plot(saved_data.time_all, saved_data.CMA_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CMA');
    title('CMA Over Time');   

    % Plot CMQ over time
    figure;
    plot(saved_data.time_all, saved_data.CMQ_all, 'b'); grid on;
    xlabel('Time (s)'); ylabel('CMQ');
    title('CMQ Over Time'); 
    
end


% ==== Parse Aerodynamic Data ====
function [ALPHA_grid, MACH_grid, MATRIX_grid] = read_coef()
    folderPath = append(pwd, '/coef_output');
    filePattern = fullfile(folderPath, 'alpha*.csv');
    csvFiles = dir(filePattern);
    N_alpha = length(csvFiles);
    
    % Coefficient fields to extract
    fields = {'Q','CN','CM','CA','CY','CLN','CLL','CL','CD','CLCD','XCP','CNA','CMA','CYB', ...
              'CLNB','CLLB','CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};
    
    % Initialize containers
    alpha_vals = zeros(N_alpha, 1);
    data_grid = struct();
    MACH_sample = [];
    
    % === Step 2: Extract alpha rows and build coefficient matrices ===
    for i = 1:N_alpha
        fileName = csvFiles(i).name;
        fullFileName = fullfile(folderPath, fileName);
        T = readtable(fullFileName);
    
        % Record alpha (assumed constant for this file)
        alpha_vals(i) = T.ALPHA(1);
    
        % Get MACH vector from the first file
        if i == 1
            MACH_sample = T.MACH';  % Column vector, assume same across all files
            for f = 1:length(fields)
                data_grid.(fields{f}) = [];  % Initialize empty
            end
        end
    
        % Append rows for each coefficient
        for f = 1:length(fields)
            coeff_values = T.(fields{f})';
            data_grid.(fields{f}) = [data_grid.(fields{f}); coeff_values];
        end
    end
    
    % === Step 3: Sort alpha and matrices ===
    [alpha_vals, sort_idx] = sort(alpha_vals);
    for f = 1:length(fields)
        data_grid.(fields{f}) = data_grid.(fields{f})(sort_idx, :);
    end

    % Create meshgrid
    [ALPHA_grid, MACH_grid] = ndgrid(alpha_vals, MACH_sample);
    MATRIX_grid = data_grid;
end

% ==== Parse Air Velocity ====
function [alt, V_a, rho] = read_air_data()
    % Read the CSV file
    folderpath = fullfile(pwd, 'air_table.csv');
    data = readtable(folderpath);
    
    % Extract arrays
    alt = data.Altitude;
    V_a = data.V_a;
    rho = data.rho;
end

% ==== Interpolation ====
function [dynamic, aero] = get_derivatives(alpha, V, z)
    global alt V_a rho rho_val;

    V_air = interp1(alt, V_a, z);
    rho_val = interp1(alt, rho, z);    
    mach_number = V / V_air;
    [dynamic, aero] = get_data_mach(alpha, mach_number);
end

function [dynamic, aero] = get_data_mach(alpha, mach)
    global ALPHA_grid MACH_grid MATRIX_grid;

    fields = {'Q','CN','CM','CA','CY','CLN','CLL','CL','CD','CLCD','XCP','CNA','CMA','CYB', ...
          'CLNB','CLLB','CNQ','CMQ','CAQ','CNAD','CMAD','CYR','CLNR','CLLR','CYP','CLNP','CLLP'};
    res = zeros(length(fields));
    for f = 1:length(fields)
        matrix = MATRIX_grid.(fields{f});
        value = interp2(MACH_grid, ALPHA_grid, matrix, mach, alpha, 'linear');
        res(f) = value;
    end
    aero = res(2:1:end);
    dynamic = res(1);
end

% ==== Save Data  ====
function [saved_data] = save_data(sim, step, step_size, X_s, M, Lf, Df, X_force, Z_force, aero_derivatives, Q)

    persistent V_all M_all Lf_all Df_all X_force_all Z_force_all;
    persistent CM_all CL_all CD_all CN_all CA_all CY_all CLN_all CLL_all;
    persistent CL_CD_all X_CP_all CNA_all CMA_all CYB_all CLNB_all CLLB_all;
    persistent CNQ_all CMQ_all CAQ_all CNAD_all CMAD_all CYR_all CLNR_all;
    persistent CLLR_all CYP_all CLNP_all CLLP_all Q_all;
    persistent x_all z_all u_all w_all theta_all q_all u_b_all;
    persistent alpha_all rho_all time_all;
    
    % Initialize arrays on the first call
    if isempty(V_all)
        V_all = [];
        M_all = [];
        Lf_all = [];
        Df_all = [];
        X_force_all = [];
        Z_force_all = [];
        CM_all = [];
        CL_all = [];
        CD_all = [];
        
        % Initialize aerodynamic coefficients arrays
        Q_all = [];
        CN_all = [];
        CA_all = [];
        CY_all = [];
        CLN_all = [];
        CLL_all = [];
        CL_CD_all = [];
        X_CP_all = [];
        CNA_all = [];
        CMA_all = [];
        CYB_all = [];
        CLNB_all = [];
        CLLB_all = [];
        CNQ_all = [];
        CMQ_all = [];
        CAQ_all = [];
        CNAD_all = [];
        CMAD_all = [];
        CYR_all = [];
        CLNR_all = [];
        CLLR_all = [];
        CYP_all = [];
        CLNP_all = [];
        CLLP_all = [];

        % Initialize state vector arrays
        x_all = [];
        z_all = [];
        u_all = [];
        w_all = [];
        theta_all = [];
        q_all = [];
u_b_all = [];
        % Initialize calculated values vector arrays     
        alpha_all = [];
        rho_all = [];
        time_all = [];
    end
    
    % Save dynamic data to arrays for the current step
    M_all(step) = M;
    Lf_all(step) = Lf;
    Df_all(step) = Df;
    X_force_all(step) = X_force;
    Z_force_all(step) = Z_force;
    
    % Save aerodynamic coefficients to arrays   
    Q_all(step) = Q;                        % Dynamic Pressure
    CN_all(step) = aero_derivatives(1);     % Longitudinal Force Coefficient
    CM_all(step) = aero_derivatives(2);     % Pitching Moment Coefficient
    CA_all(step) = aero_derivatives(3);     % Axial Force Coefficient
    CY_all(step) = aero_derivatives(4);     % Side Force Coefficient
    CLN_all(step) = aero_derivatives(5);    % Normal Force Coefficient in the lateral direction
    CLL_all(step) = aero_derivatives(6);    % Rolling Moment Coefficient
    CL_all(step) = aero_derivatives(7);     % Lift Coefficient
    CD_all(step) = aero_derivatives(8);     % Drag Coefficient
    CL_CD_all(step) = aero_derivatives(9);  % Lift-to-Drag Ratio
    X_CP_all(step) = aero_derivatives(10);  % Center of Pressure along the body length 
    CNA_all(step) = aero_derivatives(11);   % Derivative of Normal Force Coefficient with respect to Angle of Attack
    CMA_all(step) = aero_derivatives(12);   % Derivative of Pitching Moment Coefficient with respect to Angle of Attack
    CYB_all(step) = aero_derivatives(13);   % Side Force Coefficient derivative with respect to sideslip
    CLNB_all(step) = aero_derivatives(14);  % Normal Force Coefficient derivative with respect to sideslip
    CLLB_all(step) = aero_derivatives(15);  % Rolling Moment Coefficient derivative with respect to sideslip
    CNQ_all(step) = aero_derivatives(16);   % Derivative of Normal Force Coefficient with respect to pitch rate
    CMQ_all(step) = aero_derivatives(17);   % Derivative of Pitching Moment Coefficient with respect to pitch rate
    CAQ_all(step) = aero_derivatives(18);   % Derivative of Axial Force Coefficient with respect to pitch rate
    CNAD_all(step) = aero_derivatives(19);  % Derivative of Normal Force Coefficient with respect to angle of attack
    CMAD_all(step) = aero_derivatives(20);  % Derivative of Pitching Moment Coefficient with respect to angle of attack
    CYR_all(step) = aero_derivatives(21);   % Derivative of Side Force Coefficient with respect to yaw rate
    CLNR_all(step) = aero_derivatives(22);  % Normal Force Coefficient derivative with respect to yaw rate
    CLLR_all(step) = aero_derivatives(23);  % Rolling Moment Coefficient derivative with respect to yaw rate
    CYP_all(step) = aero_derivatives(24);   % Derivative of Side Force Coefficient with respect to roll rate
    CLNP_all(step) = aero_derivatives(25);  % Normal Force Coefficient derivative with respect to roll rate
    CLLP_all(step) = aero_derivatives(26);  % Rolling Moment Coefficient derivative with respect to roll rate

    % Save state vector data to arrays for the current step
    x_all(step) = X_s(1);
    z_all(step) = X_s(2);
    u_all(step) = X_s(3);
    w_all(step) = X_s(4);
    theta_all(step) = X_s(5);
    q_all(step) = X_s(6); 

    [u_b, w_b] = transform(X_s(3), X_s(4), X_s(5), "I2B");
    u_b_all(step) = u_b;
    % Save calculated values data to arrays for the current step 
    V = sqrt(X_s(3)^2 + X_s(4)^2);
    V_all(step) = V;
    alpha_all(step) = atan2(w_b, u_b) * 180/pi;
    rho_all(step) = 2*Q/V^2;  
    time_all(step) = (step-1) *step_size;

    if sim.flag
    % Create a structure to hold the saved data for easy access
    saved_data = struct('V_all', V_all, 'M_all', M_all, 'Lf_all', Lf_all, ...
                        'Df_all', Df_all, 'X_force_all', X_force_all, ...
                        'Z_force_all', Z_force_all, 'CM_all', CM_all, ...
                        'CL_all', CL_all, 'CD_all', CD_all, ...
                        'CN_all', CN_all, 'CA_all', CA_all, 'CY_all', CY_all, ...
                        'CLN_all', CLN_all, 'CLL_all', CLL_all, 'CL_CD_all', CL_CD_all, ...
                        'X_CP_all', X_CP_all, 'CNA_all', CNA_all, 'CMA_all', CMA_all, ...
                        'CYB_all', CYB_all, 'CLNB_all', CLNB_all, 'CLLB_all', CLLB_all, ...
                        'CNQ_all', CNQ_all, 'CMQ_all', CMQ_all, 'CAQ_all', CAQ_all, ...
                        'CNAD_all', CNAD_all, 'CMAD_all', CMAD_all, 'CYR_all', CYR_all, ...
                        'CLNR_all', CLNR_all, 'CLLR_all', CLLR_all, 'CYP_all', CYP_all, ...
                        'CLNP_all', CLNP_all, 'CLLP_all', CLLP_all, 'Q_all', Q_all,...
                        'x_all', x_all, 'z_all', z_all, 'u_all',u_all, 'u_b_all',u_b_all, 'w_all', w_all, ...
                        'theta_all', theta_all, 'q_all', q_all, 'alpha_all', alpha_all, 'rho_all', rho_all, ...
                        'time_all', time_all);
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

function [T_combined, f_combined, t_combined] = calculate_all_alpha_periods(saved_data, t_start, t_end)
    % Extract time and alpha arrays
    time = saved_data.time_all;
    alpha = saved_data.alpha_all;

    % Limit to specified range
    idx = time >= t_start & time <= t_end;
    t_range = time(idx);
    alpha_range = alpha(idx);

    % === Top-to-Top (peaks) ===
    [pks_top, locs_top] = findpeaks(alpha_range, t_range);
    T_top = diff(locs_top);
    f_top = 1 ./ T_top;
    t_top = 0.5 * (locs_top(1:end-1) + locs_top(2:end));

    % === Bottom-to-Bottom (valleys) ===
    [pks_bot, locs_bot] = findpeaks(-alpha_range, t_range);  % negate to find valleys
    T_bot = diff(locs_bot);
    f_bot = 1 ./ T_bot;
    t_bot = 0.5 * (locs_bot(1:end-1) + locs_bot(2:end));

    % === Combine both ===
    T_combined = [T_top, T_bot];
    f_combined = [f_top, f_bot];
    t_combined = [t_top, t_bot];

    % Sort by time for plotting
    [t_combined, sort_idx] = sort(t_combined);
    T_combined = T_combined(sort_idx);
    f_combined = f_combined(sort_idx);

    % === Print results ===
    fprintf("Top-to-Top and Bottom-to-Bottom Frequencies (%.2f to %.2f s):\n", t_start, t_end);
    fprintf("-------------------------------------------------------------\n");
    for i = 1:length(T_combined)
        fprintf("t = %.4f s | Period = %.5f s | Frequency = %.5f Hz\n", ...
            t_combined(i), T_combined(i), f_combined(i));
    end

    % === Plot Alpha with Peaks and Valleys ===
    figure;
    plot(t_range, alpha_range, 'b'); hold on;
    plot(locs_top, pks_top, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Peaks');
    plot(locs_bot, -pks_bot, 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Valleys');
    title('Alpha with Detected Peaks and Valleys');
    xlabel('Time (s)'); ylabel('Alpha (deg)'); legend(); grid on;

    % === Plot Periods ===
    figure;
    plot(t_combined, T_combined, 'k-o');
    title('Alpha Periods (Top-to-Top & Bottom-to-Bottom)');
    xlabel('Time (s)'); ylabel('Period (s)'); grid on;

    % === Plot Frequencies ===
    figure;
    plot(t_combined, f_combined, 'm-s');
    title('Alpha Frequencies (Top-to-Top & Bottom-to-Bottom)');
    xlabel('Time (s)'); ylabel('Frequency (Hz)'); grid on;
end



function [omega_n_all, zeta_all, omega_d_all, freq_Hz_all, period_all] = ...
         compute_pitch_frequency_all_steps(saved_data, Iyy, S, l)
global zeta
    N = length(saved_data.time_all);  % number of time steps
    omega_n_all = zeros(N, 1);
    zeta_all = zeros(N, 1);
    omega_d_all = zeros(N, 1);
    freq_Hz_all = zeros(N, 1);
    period_all = zeros(N, 1);

    for i = 1:N
        % Extract values at this step
        CMQ = saved_data.CMQ_all(i);
        CMAD = saved_data.CMAD_all(i);
        CMA = saved_data.CMA_all(i);
        Q_dyn = saved_data.Q_all(i);
        V = saved_data.V_all(i);
        % Dimensional derivatives
        M_alpha = CMA * Q_dyn * S * l/Iyy;
        % M_q     = CMQ * Q_dyn * S * l^2 / (2 * V*Iyy);
        % M_da    = CMAD * Q_dyn * S * l^2 / (2 * V*Iyy);
        M_q     = CMQ * S * l / (Iyy);
        M_da    = CMAD * S * l / (Iyy);

        % Check for valid conditions
        if M_alpha >= 0 || V == 0
            omega_n = NaN;
            zeta = NaN;
            omega_d = NaN;
            freq_Hz = NaN;
            T = NaN;
        else
            omega_n = sqrt(-M_alpha);
            zeta = -(M_q + M_da) / (2 * sqrt(-M_alpha));
            % zeta = -M_q / (2*omega_n);
            omega_d = omega_n * sqrt(1 - zeta^2);
            freq_Hz = omega_d / (2 * pi);
            T = 2 * pi / omega_d;
        end

        % Store
        omega_n_all(i) = omega_n;
        zeta_all(i) = zeta;
        omega_d_all(i) = omega_d;
        freq_Hz_all(i) = freq_Hz;
        period_all(i) = T;
    end
end


function compare_alpha_frequencies(saved_data, Iyy, S, l, t_start, t_end)
    % Get analytical solution from every time step
    [omega_n_all, zeta_all, omega_d_all, freq_Hz_all, period_all] = compute_pitch_frequency_all_steps(saved_data, Iyy, S, l);

    % Get numerical peak-to-peak frequency data
    [~, f_list, t_mid] = calculate_all_alpha_periods(saved_data, t_start, t_end);

    % Plot both on the same figure
    figure;
    hold on;
    plot(saved_data.time_all, freq_Hz_all, 'b-', 'DisplayName', 'Analytical Frequency (Hz)');
    plot(t_mid, f_list, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Simulated Frequency (Hz)');
    xlabel('Time (s)');
    ylabel('Frequency (Hz)');
    title('Simulated vs Analytical Alpha Frequencies');
    legend('Location', 'best');
    grid on;
end


function compare_analytical_vs_simulated_alpha(saved_data, omega_n, omega_d, zeta)

    alpha0  = saved_data.alpha_all(1);
    omega_n0 = omega_n(1);
    omega_d0 = omega_d(1);
    zeta0    = zeta(1);
    phi = atan2(-sqrt(1 - zeta0^2), -zeta0);

    t_vec = saved_data.time_all;

    % Ensure column vector for t_vec to match others
    if isrow(t_vec)
        t_vec = t_vec';
    end

    % Analytical free response: initial condition alpha0, no input
    exp_term         = exp(-zeta0 * omega_n0 * t_vec);
    oscillation_term = cos(omega_d0 * t_vec) + (zeta0/sqrt(1-zeta0^2)) * sin(omega_d0 * t_vec);
    alpha_analytic   = alpha0 * exp_term .* oscillation_term;

    % Plot: Simulated vs Analytical
    figure;
    plot(saved_data.time_all, saved_data.alpha_all, 'r-', 'LineWidth', 1.5); hold on;
    plot(saved_data.time_all, alpha_analytic, 'b--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('\alpha (deg)');
    title('Simulated vs Analytical Alpha Response (Free Decay)');
    legend('Simulated \alpha(t)', 'Analytical \alpha(t)', 'Location', 'best');
    grid on;
end


