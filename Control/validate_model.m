%% validate_model.m
% 시뮬레이션(PID_laptop) vs 실험(E5_TEST 최신) 외란 응답 비교
% 전제: init_pid_sim에서 Ts, Ki, 외란을 실험과 같게 설정

init_pid_sim;
out = sim('PID_laptop');

[ts, Xs] = get_log(out, 'log_X');
[~,  Ys] = get_log(out, 'log_Y');

% 최신 실험 결과
data_dir = fullfile(project_root, 'Main', 'DATA');
f = dir(fullfile(data_dir, 'E5_TEST_*.mat'));
[~, i] = max([f.datenum]);
E = load(fullfile(f(i).folder, f(i).name));
fprintf('실험 파일: %s (Ki %.1f, 외란 [%.2f %.2f] mm)\n', f(i).name, E.cfg.Ki, E.cfg.d_mm);
te = E.L.t - E.cfg.t_dist;                 % 외란 시각을 0으로
tsim = ts - step_time;

% 지표 (외란 후, 2 % 밴드)
d = norm(dcmd_step_mm);
[pk_s, st_s] = metric(tsim, sqrt(Xs.^2 + Ys.^2), d);
[pk_e, st_e] = metric(te, sqrt(sum(E.L.est.^2, 2)), d);
fprintf('            최대 오차   정착 시간(2 %%)\n');
fprintf('시뮬레이션  %.3f mm    %.0f ms\n', pk_s, 1e3*st_s);
fprintf('실험        %.3f mm    %.0f ms\n', pk_e, 1e3*st_e);

figure('Name', 'Model validation', 'Color', 'w');
plot(1e3*tsim, Xs, '-', 'LineWidth', 1.5); hold on; grid on;
plot(1e3*te, E.L.est(:,1), '.-');
xlim([-50 400]); xline(0, 'k--');
xlabel('외란 후 시간 [ms]'); ylabel('X [mm]');
legend('시뮬레이션 X', '실험 X̂', 'Location', 'best');
title(sprintf('외란 응답 비교 (Ki = %.0f /s, Ts = %.1f ms)', Ki_x*J(1,1), 1e3*Ts));

%% local functions
function [t, v] = get_log(out, name)
s = out.get(name);
if isa(s, 'timeseries')
    t = s.Time;  v = squeeze(s.Data);
elseif isstruct(s)
    t = s.time;  v = squeeze(s.signals.values);
else
    t = out.tout;  v = s;
end
t = t(:);  v = v(:);
end

function [pk, st] = metric(t, err, d)
m = t >= 0;  t = t(m);  err = err(m);
pk = max(err);
out_idx = find(err > 0.02*d, 1, 'last');
if isempty(out_idx), st = 0;
elseif out_idx < numel(t), st = t(out_idx + 1);
else, st = NaN;
end
end