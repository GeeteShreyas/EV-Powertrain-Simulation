%% EV Powertrain & Battery Performance Simulation
% MATLAB-based longitudinal EV model using the WLTC Class 3 driving cycle
% The model evaluates:
%   - Vehicle forces
%   - Wheel power
%   - Battery power demand
%   - Regenerative braking
%   - Battery state of charge (SOC)
%   - Energy consumption
%   - Sensitivity to vehicle mass, Cd and Crr
%
% Model assumptions:
%   - Flat road
%   - Constant motor and inverter efficiency
%   - Constant regenerative braking efficiency
%   - No auxiliary electrical loads
%   - No battery thermal or degradation model

% Initial vehicle parameters

% Vehicle
m = 1700;          % kg
Cd = 0.28;         
A = 2.3;           % m^2
Crr = 0.010;       

% Environment
rho = 1.225;       % kg/m^3
g = 9.81;           % m/s^2

% Powertrain
eta_motor = 0.92;   %efficiency 
eta_inverter = 0.97; %efficiency 

% Battery
E_battery = 60;    % kWh

% Regenerative braking
eta_regen = 0.70;

%% Driving cycle input
drive_cycle = readmatrix("WLTC_Class3.csv");

t = drive_cycle(:,1);
v_kmh = drive_cycle(:,2);

% Convert speed from km/h to m/s
v = v_kmh / 3.6;

% Time interval between consecutive data points
dt_cycle = [0; diff(t)];

%% Calculate acceleration
a = [0; diff(v) ./ diff(t)];

%% Calculate forces

F_drag = 0.5 * rho * Cd * A .* v.^2;

F_roll = Crr * m * g * ones(size(t));

F_acc = m .* a;

F_total = F_drag + F_roll + F_acc;

%% Calculate wheel power
P_wheel = F_total .* v;

%% Battery power with regenerative braking

eta_total = eta_motor * eta_inverter;
P_battery = zeros(size(P_wheel));

for i = 1:length(P_wheel)

    if P_wheel(i) >= 0
        % Driving: battery supplies power
        P_battery(i) = P_wheel(i) / eta_total;

    else
        % Braking: battery receives recovered power
        P_battery(i) = P_wheel(i) * eta_regen;
    end

end

%% Energy calculation

% Convert battery power from W to kW
P_battery_kW = P_battery / 1000;

E_step = P_battery_kW .* dt_cycle / 3600;

% Cumulative battery energy [kWh]
E_battery_used = cumsum(E_step);

% Total net energy consumed
E_net = E_battery_used(end);

fprintf('\nEnergy Results\n');
fprintf('----------------------------\n');
fprintf('Net battery energy used: %.3f kWh\n', E_net);

% Positive battery power = energy supplied by battery
P_discharge = max(P_battery, 0);

% Negative battery power = regenerative braking
P_regen = min(P_battery, 0);


% Energy discharged from battery [kWh]
E_discharge = sum(P_discharge .* dt_cycle) / 3.6e6;

% Energy recovered into battery [kWh]
E_regen = -sum(P_regen .* dt_cycle) / 3.6e6;

% Net energy consumption
E_net_check = E_discharge - E_regen;

fprintf('\nEnergy Breakdown\n');
fprintf('----------------------------\n');
fprintf('Energy discharged: %.3f kWh\n', E_discharge);
fprintf('Energy recovered:  %.3f kWh\n', E_regen);
fprintf('Net energy:        %.3f kWh\n', E_net_check);

regen_fraction = E_regen / E_discharge * 100;  % Regenerative energy recovered as a fraction of battery discharge

fprintf('Regenerative recovery: %.2f %%\n', ...
    regen_fraction);

%% Battery State of Charge (SOC)

SOC_initial = 100;          % Initial SOC [%]
SOC = SOC_initial - (E_battery_used / E_battery) * 100;
%% Energy Breakdown by Vehicle Dynamics

% Power associated with each force component [W]
P_drag = F_drag .* v;
P_roll = F_roll .* v;
P_acc  = F_acc  .* v;


% Convert power-time integral from J to kWh
J_to_kWh = 1 / 3.6e6;

% Aerodynamic drag energy
E_drag = sum(P_drag .* dt_cycle) * J_to_kWh;

% Rolling resistance energy
E_roll = sum(P_roll .* dt_cycle) * J_to_kWh;

% Positive acceleration energy
E_acc_positive = sum(max(P_acc,0) .* dt_cycle) * J_to_kWh;

% Negative acceleration energy
E_acc_negative = -sum(min(P_acc,0) .* dt_cycle) * J_to_kWh;

fprintf('\nVehicle Energy Breakdown\n');
fprintf('----------------------------\n');
fprintf('Aerodynamic drag energy: %.3f kWh\n', E_drag);
fprintf('Rolling resistance energy: %.3f kWh\n', E_roll);
fprintf('Positive acceleration energy: %.3f kWh\n', ...
    E_acc_positive);
fprintf('Braking/deceleration energy: %.3f kWh\n', ...
    E_acc_negative);

%% Powertrain Losses

% Positive wheel power
P_wheel_positive = max(P_wheel,0);

% Battery power required to produce positive wheel power
P_battery_drive = P_wheel_positive / eta_total;

% Motor + inverter losses during driving
P_powertrain_loss = P_battery_drive - P_wheel_positive;

E_powertrain_loss = ...
    sum(P_powertrain_loss .* dt_cycle) * J_to_kWh;

fprintf('Powertrain losses: %.3f kWh\n', ...
    E_powertrain_loss);

%% Consistent Energy Accounting

% Positive wheel power = mechanical energy required at wheels
P_wheel_drive = max(P_wheel, 0);

% Negative wheel power = mechanical energy available during braking
P_wheel_brake = min(P_wheel, 0);

% Energy at wheels during driving [kWh]
E_wheel_drive = sum(P_wheel_drive .* dt_cycle) / 3.6e6;

% Mechanical energy available during braking [kWh]
E_wheel_brake = -sum(P_wheel_brake .* dt_cycle) / 3.6e6;

% Battery energy supplied during driving [kWh]
E_battery_discharge = ...
    sum(max(P_battery,0) .* dt_cycle) / 3.6e6;

% Battery energy recovered through regeneration [kWh]
E_battery_regen = ...
    -sum(min(P_battery,0) .* dt_cycle) / 3.6e6;

% Net battery energy
E_battery_net = E_battery_discharge - E_battery_regen;

fprintf('\nConsistent Energy Accounting\n');
fprintf('----------------------------\n');
fprintf('Wheel energy during driving: %.3f kWh\n', ...
    E_wheel_drive);

fprintf('Mechanical braking energy: %.3f kWh\n', ...
    E_wheel_brake);

fprintf('Battery energy discharged: %.3f kWh\n', ...
    E_battery_discharge);

fprintf('Battery energy recovered: %.3f kWh\n', ...
    E_battery_regen);

fprintf('Net battery energy: %.3f kWh\n', ...
    E_battery_net);

%% Plot WLTC Class 3
figure;

plot(t, v_kmh, 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Vehicle Speed [km/h]');
title('WLTC Class 3 Driving Cycle');

grid on;
%% Plot SOC
figure;

plot(t, SOC, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('State of Charge [%]');
title('Battery State of Charge');
grid on;

%% Display final SOC

fprintf('Initial SOC: %.2f %%\n', SOC_initial);
fprintf('Final SOC: %.4f %%\n', SOC(end));

%% Distance travelled

distance = cumtrapz(t, v);

distance_km = distance / 1000;

fprintf('Distance travelled: %.3f km\n', distance_km(end));

%% Energy Consumption

energy_consumption = (E_net / distance_km(end)) * 100;

fprintf('Energy consumption: %.2f kWh/100 km\n', ...
    energy_consumption);

%% Plot acceleration

figure;

plot(t, a, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
title('Vehicle Acceleration');
grid on;


%% Plot total force

figure;

plot(t, F_total, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Force [N]');
title('Total Vehicle Force');
grid on;


%% Plot wheel power

figure;

plot(t, P_wheel/1000, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Wheel Power [kW]');
title('Wheel Power');
grid on;


%% Plot battery power

figure;

plot(t, P_battery/1000, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Battery Power [kW]');
title('Battery Power');
grid on;

%% Plot cumulative battery energy

figure;

plot(t, E_battery_used, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Net Battery Energy [kWh]');
title('Cumulative Net Battery Energy ');
grid on;


fprintf('Cycle duration: %.1f s\n', t(end));
fprintf('Maximum speed: %.1f km/h\n', max(v_kmh));
fprintf('Distance: %.2f km\n', trapz(t,v)/1000);
fprintf('Number of data points: %d\n', length(t));


%% Mass Sensitivity Analysis

mass_values = 1500:100:2000;

energy_consumption_mass = zeros(size(mass_values));

for j = 1:length(mass_values)

    % Change vehicle mass
    m_test = mass_values(j);

    % Recalculate rolling resistance
    F_roll_test = Crr * m_test * g * ones(size(t));

    % Recalculate acceleration force
    F_acc_test = m_test .* a;

    % Total force
    F_total_test = F_drag + F_roll_test + F_acc_test;

    % Wheel power
    P_wheel_test = F_total_test .* v;

    % Battery power
    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0

            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;

        else

            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;

        end

    end

    % Energy
    P_battery_test_kW = P_battery_test / 1000;

    E_step_test = ...
        P_battery_test_kW .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    % Energy consumption
    energy_consumption_mass(j) = ...
        E_net_test / distance_km(end) * 100;

end

%% Mass vs Energy Consumption

figure;

plot(mass_values, energy_consumption_mass, ...
    '-o', 'LineWidth', 1.5);

xlabel('Vehicle Mass [kg]');
ylabel('Energy Consumption [kWh/100 km]');
title('Effect of Vehicle Mass on EV Energy Consumption');

grid on;

fprintf('\nMass Sensitivity Results\n');
fprintf('----------------------------\n');

for j = 1:length(mass_values)

    fprintf('%d kg --> %.2f kWh/100 km\n', ...
        mass_values(j), ...
        energy_consumption_mass(j));

end

%% Aerodynamic Drag Sensitivity Analysis

Cd_values = 0.20:0.02:0.34;

energy_consumption_Cd = zeros(size(Cd_values));

for j = 1:length(Cd_values)

    Cd_test = Cd_values(j);

    % Aerodynamic drag
    F_drag_test = ...
        0.5 * rho * Cd_test * A .* v.^2;

    % Rolling resistance
    F_roll_test = ...
        Crr * m * g * ones(size(t));

    % Acceleration force
    F_acc_test = m .* a;

    % Total force
    F_total_test = ...
        F_drag_test + F_roll_test + F_acc_test;

    % Wheel power
    P_wheel_test = F_total_test .* v;

    % Battery power
    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0

            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;

        else

            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;

        end

    end

    % Energy
    P_battery_test_kW = P_battery_test / 1000;

    E_step_test = ...
        P_battery_test_kW .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    % Energy consumption
    energy_consumption_Cd(j) = ...
        E_net_test / distance_km(end) * 100;

end

%% Cd vs Energy Consumption

figure;

plot(Cd_values, energy_consumption_Cd, ...
    '-o', 'LineWidth', 1.5);

xlabel('Drag Coefficient, C_d');
ylabel('Energy Consumption [kWh/100 km]');
title('Effect of Aerodynamic Drag on EV Energy Consumption');

grid on;

fprintf('\nCd Sensitivity Results\n');
fprintf('----------------------------\n');

for j = 1:length(Cd_values)

    fprintf('Cd = %.2f --> %.2f kWh/100 km\n', ...
        Cd_values(j), ...
        energy_consumption_Cd(j));

end

%% Rolling Resistance Sensitivity Analysis

Crr_values = 0.006:0.002:0.014;

energy_consumption_Crr = zeros(size(Crr_values));

for j = 1:length(Crr_values)

    Crr_test = Crr_values(j);

    % Aerodynamic drag
    F_drag_test = ...
        0.5 * rho * Cd * A .* v.^2;

    % Rolling resistance
    F_roll_test = ...
        Crr_test * m * g * ones(size(t));

    % Acceleration force
    F_acc_test = m .* a;

    % Total force
    F_total_test = ...
        F_drag_test + F_roll_test + F_acc_test;

    % Wheel power
    P_wheel_test = F_total_test .* v;

    % Battery power
    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0

            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;

        else

            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;

        end

    end

    % Energy calculation
    P_battery_test_kW = P_battery_test / 1000;

    E_step_test = ...
        P_battery_test_kW .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    % Energy consumption
    energy_consumption_Crr(j) = ...
        E_net_test / distance_km(end) * 100;

end

%% Crr vs Energy Consumption

figure;

plot(Crr_values, energy_consumption_Crr, ...
    '-o', 'LineWidth', 1.5);

xlabel('Rolling Resistance Coefficient, C_{rr}');
ylabel('Energy Consumption [kWh/100 km]');
title('Effect of Rolling Resistance on EV Energy Consumption');

grid on;

fprintf('\nCrr Sensitivity Results\n');
fprintf('----------------------------\n');

for j = 1:length(Crr_values)

    fprintf('Crr = %.3f --> %.2f kWh/100 km\n', ...
        Crr_values(j), ...
        energy_consumption_Crr(j));

end


%% Normalized Sensitivity Analysis

% Baseline values
m_base = m;
Cd_base = Cd;
Crr_base = Crr;

% Parameter change
change = 0.10;     % +/- 10%

%% Function to calculate energy consumption

% ---- Mass ----
m_low = m_base * (1 - change);
m_high = m_base * (1 + change);

energy_mass = zeros(1,2);
mass_test = [m_low, m_high];

for j = 1:2

    m_test = mass_test(j);

    F_drag_test = 0.5 * rho * Cd_base * A .* v.^2;
    F_roll_test = Crr_base * m_test * g * ones(size(t));
    F_acc_test = m_test .* a;

    F_total_test = ...
        F_drag_test + F_roll_test + F_acc_test;

    P_wheel_test = F_total_test .* v;

    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0
            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;
        else
            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;
        end

    end

    E_step_test = ...
        (P_battery_test / 1000) .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    energy_mass(j) = ...
        E_net_test / distance_km(end) * 100;

end


%% ---- Cd ----

Cd_low = Cd_base * (1 - change);
Cd_high = Cd_base * (1 + change);

energy_Cd = zeros(1,2);
Cd_test_values = [Cd_low, Cd_high];

for j = 1:2

    Cd_test = Cd_test_values(j);

    F_drag_test = ...
        0.5 * rho * Cd_test * A .* v.^2;

    F_roll_test = ...
        Crr_base * m_base * g * ones(size(t));

    F_acc_test = m_base .* a;

    F_total_test = ...
        F_drag_test + F_roll_test + F_acc_test;

    P_wheel_test = F_total_test .* v;

    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0
            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;
        else
            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;
        end

    end

    E_step_test = ...
        (P_battery_test / 1000) .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    energy_Cd(j) = ...
        E_net_test / distance_km(end) * 100;

end


%% ---- Crr ----

Crr_low = Crr_base * (1 - change);
Crr_high = Crr_base * (1 + change);

energy_Crr = zeros(1,2);
Crr_test_values = [Crr_low, Crr_high];

for j = 1:2

    Crr_test = Crr_test_values(j);

    F_drag_test = ...
        0.5 * rho * Cd_base * A .* v.^2;

    F_roll_test = ...
        Crr_test * m_base * g * ones(size(t));

    F_acc_test = m_base .* a;

    F_total_test = ...
        F_drag_test + F_roll_test + F_acc_test;

    P_wheel_test = F_total_test .* v;

    P_battery_test = zeros(size(P_wheel_test));

    for i = 1:length(P_wheel_test)

        if P_wheel_test(i) >= 0
            P_battery_test(i) = ...
                P_wheel_test(i) / eta_total;
        else
            P_battery_test(i) = ...
                P_wheel_test(i) * eta_regen;
        end

    end

    E_step_test = ...
        (P_battery_test / 1000) .* dt_cycle / 3600;

    E_net_test = sum(E_step_test);

    energy_Crr(j) = ...
        E_net_test / distance_km(end) * 100;

end

%% Calculate Normalized Sensitivity

E_base = energy_consumption;   % baseline energy consumption [kWh/100 km]

% Percentage change in energy consumption
mass_pct_change = ...
    ((energy_mass(2) - energy_mass(1)) / (2 * E_base)) * 100;

Cd_pct_change = ...
    ((energy_Cd(2) - energy_Cd(1)) / (2 * E_base)) * 100;

Crr_pct_change = ...
    ((energy_Crr(2) - energy_Crr(1)) / (2 * E_base)) * 100;

% Normalized sensitivity
S_mass = mass_pct_change / 10;
S_Cd = Cd_pct_change / 10;
S_Crr = Crr_pct_change / 10;

fprintf('\nNormalized Sensitivity Analysis\n');
fprintf('--------------------------------\n');

fprintf('Mass: %.3f\n', S_mass);
fprintf('Cd:   %.3f\n', S_Cd);
fprintf('Crr:  %.3f\n', S_Crr);

%% Sensitivity Comparison

sensitivity_values = [S_mass, S_Cd, S_Crr];

figure;

bar(sensitivity_values);

xticklabels({'Mass','C_d','C_{rr}'});
ylabel('Normalized Sensitivity');
title('Normalized Sensitivity of EV Energy Consumption');

grid on;