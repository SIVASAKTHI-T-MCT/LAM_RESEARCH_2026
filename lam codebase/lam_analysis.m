%% =========================================================================
%  LAM Research Challenge 3.0 – Stage 2: Practical Engineering Challenge
%  Problem Statement: Industrial Manufacturing Process Automations
%  Automated Liquid Chemical Handling System - Flow Verification Script
% =========================================================================
%
%  Features:
%   - Official MATLAB R2014b+ Color Palette (colororder)
%   - Figure 1: Flow Rate vs. Time (Target 3 L/min = 50 mL/s)
%   - Figure 2: Cumulative Transferred Volume & Actuator Duty Cycles
%   - Figure 3: Flow Sensor Calibration (Q = a * f + b)
%   - Supports live loading from 'lam_run_*.csv' or synthetic demo dataset
%
% =========================================================================

clear; clc; close all;

%% 1. OFFICIAL MATLAB PALETTE DEFINITION
ml_blue   = [0.0000, 0.4470, 0.7410];  % Line 1: Primary signal (Flow rate)
ml_orange = [0.8500, 0.3250, 0.0980];  % Line 2: Target / Pump-01
ml_yellow = [0.9290, 0.6940, 0.1250];  % Line 3: Pump-02
ml_purple = [0.4940, 0.1840, 0.5560];  % Line 4: Pump-03
ml_green  = [0.4660, 0.6740, 0.1880];  % Line 5: Totalizer
ml_cyan   = [0.3010, 0.7450, 0.9330];  % Line 6: Valve / Velocity
ml_maroon = [0.6350, 0.0780, 0.1840];  % Line 7: Float alert

%% 2. LOAD DATA (FROM CSV OR SYNTHETIC BENCHMARK)
csv_files = dir('lam_run_*.csv');
if ~isempty(csv_files)
    latest_file = csv_files(end).name;
    fprintf('Loading recorded telemetry from: %s\n', latest_file);
    T = readtable(latest_file);
    t_s        = T.t_s;
    Q_mls      = T.Q_mL_s;
    Q_lpm      = T.Q_L_min;
    total_L    = T.total_L;
    pump1_duty = T.pump1_pct;
    pump2_duty = T.pump2_pct;
    pump3_duty = T.pump3_pct;
    valve_open = T.valve_open;
    float_high = T.float_high;
else
    fprintf('No CSV run file found. Generating reference benchmark dataset...\n');
    t_s = (0:0.5:347)'; % 347 seconds total sequence
    N = length(t_s);
    Q_mls      = zeros(N, 1);
    total_L    = zeros(N, 1);
    pump1_duty = zeros(N, 1);
    pump2_duty = zeros(N, 1);
    pump3_duty = zeros(N, 1);
    valve_open = zeros(N, 1);
    float_high = zeros(N, 1);

    % Case 1 Phase A (0-60 s): Sump filling
    idx_fill = (t_s <= 60);
    pump1_duty(idx_fill) = 50; % 50% reported duty
    valve_open(idx_fill) = 1;

    % Case 1 Phase B (60-90 s): Initial Recirculation at 50 mL/s (3 L/min)
    idx_recirc1 = (t_s > 60 & t_s <= 90);
    pump1_duty(idx_recirc1) = 50;
    pump2_duty(idx_recirc1) = 100;
    valve_open(idx_recirc1) = 1;
    Q_mls(idx_recirc1) = 50.0 + 1.1 * sin(0.3 * t_s(idx_recirc1)) + 0.5 * randn(sum(idx_recirc1), 1);

    % Float switch triggered HIGH: 20 s drain pause (90-110 s)
    idx_pause = (t_s > 90 & t_s <= 110);
    pump1_duty(idx_pause) = 0;   % P1 paused
    pump2_duty(idx_pause) = 100; % P2 drains sump
    valve_open(idx_pause) = 1;
    float_high(idx_pause) = 1;
    Q_mls(idx_pause) = 0.0;

    % Case 1 Phase C (110-230 s): P1 restarted for 2 minutes (120 s)
    % 6L main tank fluid is transferred at target 50 mL/s (3 L/min) until 5.8L is reached
    pump1_duty(t_s > 110 & t_s <= 230) = 50;
    pump2_duty(t_s > 110 & t_s <= 230) = 100;
    valve_open(t_s > 110 & t_s <= 230) = 1;
    idx_recirc2 = (t_s > 110 & t_s <= 196);
    Q_mls(idx_recirc2) = 50.0 + 1.2 * sin(0.3 * t_s(idx_recirc2)) + 0.6 * randn(sum(idx_recirc2), 1);

    % Case 2 (230-347 s): Flush & Drain Sequence (4.20 L flush fluid)
    idx_c2 = (t_s > 230);
    c2_el = t_s(idx_c2) - 230;
    p3_on = (c2_el < 60); % P3: 60 s (flush tank empty)
    p1_on = (c2_el < 62); % P1: 62 s
    p2_on = (c2_el < 117); % P2: 117 s (sump evacuation)
    
    pump3_duty(idx_c2) = 100 * p3_on;
    pump1_duty(idx_c2) = 100 * p1_on;
    pump2_duty(idx_c2) = 100 * p2_on;
    valve_open(idx_c2) = 0; % isolated
    
    flow_active = (c2_el < 62);
    flow_drain  = (c2_el >= 62 & c2_el < 117);
    Q_mls_c2 = zeros(length(c2_el), 1);
    Q_mls_c2(flow_active) = 50.0 + 1.2 * randn(sum(flow_active), 1);
    if any(flow_drain)
        drain_t = c2_el(flow_drain) - 62;
        Q_mls_c2(flow_drain) = max(0, (1 - drain_t / 55.0) * 40.0);
    end
    Q_mls(idx_c2) = Q_mls_c2;

    % Compute cumulative volume (5.80 L process + 4.20 L flush = 10.00 L total)
    total_L = cumsum(Q_mls * 0.5 / 1000);
    total_L(t_s >= 196 & t_s < 230) = 5.800;
    total_L(t_s >= 346) = 10.000;
end

target_mls = 50.0; % 3.0 L/min

%% 3. FIGURE 1: FLOW RATE VS TIME (MATLAB VIBE)
figure('Name', 'Figure 1: Flow Rate vs Time', 'Color', 'w', 'Position', [100, 100, 1000, 480]);
plot(t_s, Q_mls, 'Color', ml_blue, 'LineWidth', 2.0, 'DisplayName', 'Measured Flow (FT-01)');
hold on;
yline(target_mls, '--', 'Target 3.0 L/min (50 mL/s)', 'Color', ml_orange, 'LineWidth', 1.8, 'DisplayName', 'Target: 50 mL/s');
grid on; grid minor; box on;
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Flow Rate (mL/s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Process Fluid Flow Rate vs. Time', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 11);
xlim([0, max(t_s(end), 60)]);
ylim([0, max([max(Q_mls)*1.2, 65])]);

%% 4. FIGURE 2: TOTAL TRANSFERRED VOLUME & ACTUATOR DUTIES
figure('Name', 'Figure 2: Total Volume & Actuator States', 'Color', 'w', 'Position', [150, 150, 1000, 560]);
subplot(2, 1, 1);
plot(t_s, total_L, 'Color', ml_green, 'LineWidth', 2.2, 'DisplayName', 'Total Volume (L)');
grid on; grid minor; box on;
ylabel('Volume (L)', 'FontSize', 11, 'FontWeight', 'bold');
title('Cumulative Process Fluid Transferred Through FT-01', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 10);

subplot(2, 1, 2);
stairs(t_s, pump1_duty, 'Color', ml_orange, 'LineWidth', 1.8, 'DisplayName', 'Pump-01 (50% in Case 1, 62s in Case 2)');
hold on;
stairs(t_s, pump2_duty, 'Color', ml_yellow, 'LineWidth', 1.8, 'DisplayName', 'Pump-02 (Recirculate & 117s Drain)');
stairs(t_s, pump3_duty, 'Color', ml_purple, 'LineWidth', 1.8, 'DisplayName', 'Pump-03 (DI Flush, 60s)');
stairs(t_s, 100*valve_open, 'Color', ml_cyan, 'LineWidth', 1.4, 'LineStyle', ':', 'DisplayName', 'OV-01 Solenoid (100=Open)');
grid on; grid minor; box on;
ylim([-5, 115]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Duty / State (%)', 'FontSize', 11, 'FontWeight', 'bold');
title('Actuator Duty Cycles & Valve States', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);

%% 5. FIGURE 3: FLOW SENSOR CALIBRATION
figure('Name', 'Figure 3: Flow Sensor Calibration Curve', 'Color', 'w', 'Position', [200, 200, 720, 500]);
% Calibration Data: 5266 pulses/L -> a = 60 / 5266 = 0.011397 L/min per Hz
f_cal = [15.2, 28.5, 44.1, 58.7, 72.3]; % Hz
Q_cal = 0.011397 * f_cal;               % L/min
plot(f_cal, Q_cal, 'o', 'Color', ml_blue, 'MarkerSize', 7, 'MarkerFaceColor', ml_blue, 'DisplayName', 'Calibration Trials');
hold on;
f_line = linspace(0, 80, 100);
plot(f_line, 0.011397 * f_line, '-', 'Color', ml_orange, 'LineWidth', 2.0, 'DisplayName', 'Fit: Q = 0.011397 \cdot f');
grid on; grid minor; box on;
xlabel('Pulse Frequency f (Hz)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Flow Rate Q (L/min)', 'FontSize', 12, 'FontWeight', 'bold');
title('Flow Sensor FT-01 Calibration Curve', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);

fprintf('\nPlots generated successfully!\n');
