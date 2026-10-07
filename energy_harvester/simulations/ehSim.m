function simulate_EH(audio_file)
% v1: magnet/coil EH array. Printed summary only — no plots.

    % --- load input disturbance (audio -> base acceleration) ---
    [x, fs] = audioread(audio_file);
    if size(x,2) > 1, x = mean(x,2); end        % mono
    t = (0:length(x)-1)' / fs;

    accel_scale = 10;                           % calibration knob (m/s^2 per unit audio)
    a = accel_scale * x;

    % --- define EH array (edit to add/remove) ---
    %   fn     natural freq (Hz)
    %   m      magnet mass (kg)
    %   zeta   mechanical damping ratio
    %   B      magnet field (T)
    %   N      coil turns
    %   D      coil mean diameter (m)
    %   R_coil coil resistance (ohm)
    %   R_load load resistance (ohm)
    eh(1) = struct('name','EH1', 'fn', 50,  ...
        'm',10e-3, 'zeta',0.05, 'B',0.5, 'N',200, 'D',0.015, ...
        'R_coil',10, 'R_load',100);
    eh(2) = eh(1);  eh(2).name = 'EH2';  eh(2).fn = 100;
    eh(3) = eh(1);  eh(3).name = 'EH3';  eh(3).fn = 150;

    % --- simulate each EH and print one line ---
    fprintf('\n%-6s %-8s %-14s %-14s %-14s\n', ...
        'Name', 'fn (Hz)', 'E_total (J)', 'P_avg (W)', 'P_peak (W)');
    fprintf('%s\n', repmat('-', 1, 60));
    for k = 1:length(eh)
        [P, E] = sim_mag_coil(eh(k), a, t);
        fprintf('%-6s %-8g %-14.3e %-14.3e %-14.3e\n', ...
            eh(k).name, eh(k).fn, E(end), mean(P), max(P));
    end
end

% ------------------------------------------------------------------
function [P, E] = sim_mag_coil(cfg, a, t)
% Forward-Euler integration of the magnet-in-coil EOM:
%   m*xdd + (c_mech + BL^2/R_tot)*xd + k*x = -m*a(t)
%   V_load = BL * xd * R_load / R_tot

    wn    = 2*pi*cfg.fn;
    k_sp  = cfg.m * wn^2;
    c_m   = 2 * cfg.zeta * sqrt(cfg.m * k_sp);
    BL    = cfg.B * cfg.N * pi * cfg.D;
    R_tot = cfg.R_coil + cfg.R_load;
    c_tot = c_m + BL^2 / R_tot;

    dt = t(2) - t(1);
    N  = length(t);
    V_load = zeros(N,1);
    x = 0;  xd = 0;

    for i = 1:N-1
        xdd    = (-c_tot*xd - k_sp*x - cfg.m*a(i)) / cfg.m;
        xd     = xd + dt*xdd;
        x      = x  + dt*xd;
        V_load(i+1) = BL * xd * cfg.R_load / R_tot;
    end

    P = V_load.^2 / cfg.R_load;
    E = cumtrapz(t, P);
end


simulate_EH('C:\Users\aksha\Downloads\da97011f598488c3ff8cc132664d6819 (2).mov');