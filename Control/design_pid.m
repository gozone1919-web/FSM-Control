%% design_pid.m
% 기준 PID를 근거 있게 다시 정하기: 세 가지 설계안을 같은 조건으로 비교
%   A. pidtune (MATLAB 표준, 주파수영역 설계, 목표 위상 여유 60°): PI, PID
%   B. SIMC 공식 (Journal of Process Control 2003, E4 FOPDT 모델): PI
%   C. Kp × Ki 격자에서 ITAE 최소 (선형 모델 계단 응답)
%   + 지금 기준 PID (0.5, 20, 0)
% 비교: 위상 여유, 이득 여유, ITAE(선형), 그리고 Simulink(실측 잡음, seed 1~5)의
%       정착 시간, 오버슈트, 중심 흔들림
% 게인 단위: 루프 이득 (블록 값 = 값 / J), Kd [s]

init_pid_sim;                        % Ged (E4 미러 이산 모델), Ts, J 등
mdl = 'PID_laptop';
PM_target = 60;                      % [deg] pidtune 목표 위상 여유
E4_fopdt  = [0.0194 0.0106];         % [s] E4 FOPDT 피팅 [지연, 시정수]
seeds = 1:5;  r_small = [0.2 0.5];  band = 0.02;

%% 1. 설계안
opt = pidtuneOptions('PhaseMargin', PM_target);
Cpi  = pidtune(Ged, pid(1, 1, 0, 0, Ts, 'IFormula', 'BackwardEuler'), opt);
Cpid = pidtune(Ged, pid(1, 1, 1, 0.01, Ts, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler'), opt);

th = E4_fopdt(1) + Ts/2;  tau = E4_fopdt(2);         % 샘플링(ZOH) 지연 Ts/2 포함
Kc = tau / (2*th);  tI = min(tau, 8*th);              % SIMC, 목표 시정수 = 지연
Csimc = pid(Kc, Kc/tI, 0, 0, Ts, 'IFormula', 'BackwardEuler');

Kp_g = 0.1:0.1:1.0;  Ki_g = [5 10 15 20 25 30 40 50 60];
IT = NaN(numel(Ki_g), numel(Kp_g));  PM = IT;
for a = 1:numel(Kp_g)
    for b = 1:numel(Ki_g)
        C = pid(Kp_g(a), Ki_g(b), 0, 0, Ts, 'IFormula', 'BackwardEuler');
        [IT(b,a), PM(b,a)] = lin_eval(C, Ged);
    end
end
[~, k] = min(IT(:));  [b, a] = ind2sub(size(IT), k);
Cgrid = pid(Kp_g(a), Ki_g(b), 0, 0, Ts, 'IFormula', 'BackwardEuler');
Cstd  = pid(0.5, 20, 0, 0, Ts, 'IFormula', 'BackwardEuler');

names = {'기준 PID (지금)', 'pidtune PI', 'pidtune PID', 'SIMC PI', '격자 ITAE 최소'};
Cs    = {Cstd, Cpi, Cpid, Csimc, Cgrid};

%% 2. Simulink 비교 (이상적 선형 센서, 실측 잡음)
Xhat_sensor = Xs_sensor;  Yhat_sensor = Ys_sensor;
[nzx, nzy] = load_noise(data_dir);
Nn = round(Tstop/Ts) + 1;  tn = (0:Nn-1).' * Ts;
R = table();
for i = 1:numel(Cs)
    C = Cs{i};  N = 30;  if C.Tf > 0, N = 1/C.Tf; end
    [itl, pm, gm] = lin_eval(C, Ged);
    assignin('base', 'Kp_x', C.Kp/J(1,1));  assignin('base', 'Kp_y', C.Kp/J(2,2));
    assignin('base', 'Ki_x', C.Ki/J(1,1));  assignin('base', 'Ki_y', C.Ki/J(2,2));
    assignin('base', 'Kd_x', C.Kd/J(1,1));  assignin('base', 'Kd_y', C.Kd/J(2,2));
    assignin('base', 'N_x', N);  assignin('base', 'N_y', N);
    M = NaN(numel(seeds), 3);
    for s = 1:numel(seeds)
        rs = RandStream('mt19937ar', 'Seed', seeds(s));
        r = r_small(1) + diff(r_small)*rand(rs);  q = 2*pi*rand(rs);
        dvec = r*[cos(q); sin(q)];  du = J \ dvec;
        assignin('base', 'dux', 0);  assignin('base', 'duy', 0);
        assignin('base', 'dux_step', du(1));  assignin('base', 'duy_step', du(2));
        i0 = 50*(s-1);
        assignin('base', 'noise_x', [tn nzx(i0+(1:Nn))]);
        assignin('base', 'noise_y', [tn nzy(i0+(1:Nn))]);
        out = sim(mdl);
        [t, x] = get_log(out, 'log_X');  [~, y] = get_log(out, 'log_Y');
        M(s,:) = sim_metric(t - step_time, [x y], dvec, band);
    end
    R = [R; table(string(names{i}), C.Kp, C.Ki, C.Kd, pm, gm, itl, ...
        mean(M(:,1), 'omitnan'), max(M(:,2)), mean(M(:,3)), ...
        'VariableNames', {'design', 'Kp', 'Ki', 'Kd', 'PM_deg', 'GM_dB', 'ITAE', ...
        'settle_ms', 'os_pct', 'jitter_um'})]; %#ok<AGROW>
end
fprintf('\n=== 설계안 비교 (선형 센서, 실측 잡음, 외란 %.1f~%.1f mm, seed %d개) ===\n', r_small, numel(seeds));
disp(R)
save(fullfile(data_dir, sprintf('DESIGN_PID_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), ...
    'R', 'Cs', 'names', 'Kp_g', 'Ki_g', 'IT', 'PM', 'PM_target', 'E4_fopdt');

%% 3. 그림: Kp × Ki ITAE 지도 + 위상 여유 45°, 60° 선 + 설계안 위치
figure('Name', 'PI design map', 'Color', 'w');  hold on;
ITp = IT;  ITp(~isfinite(ITp)) = NaN;                 % 불안정 칸은 비움
contourf(Kp_g, Ki_g, log10(ITp), 20, 'LineColor', 'none');  colorbar;
[cc, hh] = contour(Kp_g, Ki_g, PM, [45 60], 'k', 'LineWidth', 1.2);  clabel(cc, hh);
mk = {'ko', 'ws', 'w^', 'wd', 'wp'};
for i = 1:numel(Cs), plot(Cs{i}.Kp, Cs{i}.Ki, mk{i}, 'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', names{i}); end
xlabel('Kp'); ylabel('Ki [1/s]'); title('log_{10}(ITAE), 검은 선: 위상 여유 45° / 60°');
legend(findobj(gca, 'Type', 'line'), 'Location', 'northeastoutside');

%% local functions
function [itae, pm, gm] = lin_eval(C, G)
% 선형 모델: 명령 외란 단위 계단 -> 위치 응답의 ITAE (1.5 s), 위상·이득 여유
    T = feedback(G, C);
    if ~isstable(T), itae = Inf; pm = NaN; gm = NaN; return; end
    [y, t] = step(T, 1.5);
    itae = trapz(t, t .* abs(y));
    [g, pm] = margin(C*G);  gm = 20*log10(g);
end

function m = sim_metric(t, P, dvec, band)
% [정착 시간 ms (0의 2 % 밴드), 오버슈트 %, 마지막 0.5 s 흔들림 um]
    k = t >= 0;  t = t(k);  P = P(k,:);
    d = norm(dvec);  e = sqrt(sum(P.^2, 2));
    lo = find(e > band*d, 1, 'last');
    if isempty(lo), st = 0; elseif lo < numel(t), st = 1e3*t(lo+1); else, st = NaN; end
    proj = P*(dvec/d);  [~, ip] = max(proj);
    os = max(0, -min(proj(ip:end))) / d * 100;
    w = t >= t(end) - 0.5;  Pw = P(w,:);
    jt = 1e3*sqrt(mean(sum((Pw - mean(Pw, 1)).^2, 2)));
    m = [st os jt];
end

function [t, v] = get_log(out, name)
    s = out.get(name);
    if isa(s, 'timeseries'), t = s.Time; v = squeeze(s.Data);
    elseif isstruct(s),      t = s.time; v = squeeze(s.signals.values);
    else,                    t = out.tout; v = s;
    end
    t = t(:);  v = v(:);
end

function [nx, ny] = load_noise(data_dir)
% 최신 NOISE_*.mat (noise_3_laser)를 DoS 중심 기울기로 위치 잡음 [mm]로 바꿈
    f = dir(fullfile(data_dir, 'NOISE_*.mat'));  [~, i] = max([f.datenum]);
    R = load(fullfile(f(i).folder, f(i).name));
    s.XDiff = R.Raw(:,1); s.YDiff = R.Raw(:,2); s.SUM = R.Raw(:,3);
    [~, ~, dx, dy] = estimator_dos(s, 1);
    sl = [0 0];  ax = 'XY';
    for k = 1:2
        g = dir(fullfile(data_dir, ['BEAM_SCAN_' ax(k) '_*.mat']));  [~, j] = max([g.datenum]);
        B = load(fullfile(g(j).folder, g(j).name));  m = abs(B.X(:)) <= 0.3;
        b.XDiff = B.M(m,1); b.YDiff = B.M(m,2); b.SUM = B.M(m,3);
        [~, ~, ex, ey] = estimator_dos(b, 1);
        if k == 1, p = polyfit(B.X(m), ex, 1); else, p = polyfit(B.X(m), ey, 1); end
        sl(k) = p(1);
    end
    nx = (dx - mean(dx)) / sl(1);  ny = (dy - mean(dy)) / sl(2);
end