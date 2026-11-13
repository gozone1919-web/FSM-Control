%% tune_pid.m
% PID 튜닝 P -> I -> D (오버슈트 한계 없음, 단계마다 ITAE 최소)
%   센서: 이상적 선형 (X̂ = X), 잡음: 최신 NOISE_*.mat 실측값 (noise_3_laser)
%   외란: 작은 랜덤 스텝 0.2~0.5 mm, seed 1~5 (standard.m과 같음)
%   ITAE = ∫ t·|e(t)| dt / 외란 크기   (e = 중심까지 거리, 외란 후 1.5 s)
%   함께 기록: 정착 시간, 오버슈트, 중심 흔들림(마지막 0.5 s rms)
%   게인 단위: 루프 이득 (블록 값 = 값 / J), Kd 단위 [s]

init_pid_sim;                       % Ts, FSM(E4), J 등
mdl = 'PID_laptop';

%% 1. 설정
Kp_list = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.2];
Ki_list = [5 10 15 20 25 30 40 50 60 80];
Kd_list = [0 0.0005 0.001 0.002 0.003 0.004 0.006];
N_d     = 30;                       % D 필터 계수 (N*Ts < 1)
seeds   = 1:5;
r_small = [0.2 0.5];                % [mm]
band    = 0.02;                     % 정착 밴드 (외란 대비)
std_pid = [0.5 20 0];               % 비교용 기준 PID [Kp Ki Kd]

%% 2. 선형 센서, D 필터, 실측 잡음
Xhat_sensor = Xs_sensor;  Yhat_sensor = Ys_sensor;
N_x = N_d;  N_y = N_d;
[nzx, nzy, nz_file] = load_noise(data_dir);
Nn = round(Tstop/Ts) + 1;
if numel(nzx) < Nn + 4*50, error('잡음 기록이 짧음 (%d개)', numel(nzx)); end
fprintf('\n센서: 선형, 잡음: %s (X %.2f, Y %.2f um 1σ), 외란 %.1f~%.1f mm, seed %d개\n', ...
    nz_file, 1e3*std(nzx), 1e3*std(nzy), r_small, numel(seeds));

%% 3. 단계별 튜닝
P1 = run_stage(mdl, Kp_list, 0, 0, 'Kp', seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy);
[~, b] = min(P1.itae);  Kp_sel = Kp_list(b);
fprintf('\n[1단계 P] Ki = 0, Kd = 0\n');  disp(P1)
fprintf('-> Kp = %.2f\n', Kp_sel);

I1 = run_stage(mdl, Kp_sel, Ki_list, 0, 'Ki', seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy);
[~, b] = min(I1.itae);  Ki_sel = Ki_list(b);
fprintf('\n[2단계 I] Kp = %.2f, Kd = 0\n', Kp_sel);  disp(I1)
fprintf('-> Ki = %.0f /s\n', Ki_sel);

D1 = run_stage(mdl, Kp_sel, Ki_sel, Kd_list, 'Kd', seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy);
[~, b] = min(D1.itae);  Kd_sel = Kd_list(b);
fprintf('\n[3단계 D] Kp = %.2f, Ki = %.0f\n', Kp_sel, Ki_sel);  disp(D1)
fprintf('-> Kd = %.4f s\n', Kd_sel);

%% 4. 기준 PID와 비교
S0 = run_stage(mdl, std_pid(1), std_pid(2), std_pid(3), 'Kp', seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy);
S1 = run_stage(mdl, Kp_sel, Ki_sel, Kd_sel, 'Kp', seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy);
fprintf('\n=== 비교 ===\n');
fprintf('기준 PID  (Kp %.2f, Ki %.0f, Kd %.4f): ITAE %.4f, 정착 %.0f ms, 오버슈트 %.1f %%, 흔들림 %.2f um\n', ...
    std_pid, S0.itae, S0.settle_ms, S0.os_pct, S0.jitter_um);
fprintf('P->I->D   (Kp %.2f, Ki %.0f, Kd %.4f): ITAE %.4f, 정착 %.0f ms, 오버슈트 %.1f %%, 흔들림 %.2f um\n', ...
    Kp_sel, Ki_sel, Kd_sel, S1.itae, S1.settle_ms, S1.os_pct, S1.jitter_um);

save(fullfile(data_dir, sprintf('TUNE_PID_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), ...
    'P1', 'I1', 'D1', 'S0', 'S1', 'Kp_sel', 'Ki_sel', 'Kd_sel', 'std_pid', 'N_d', 'seeds', 'r_small', 'Ts', 'nz_file');

%% 5. 그림
figure('Name', 'PID tuning P-I-D (ITAE)', 'Color', 'w', 'Position', [80 80 1300 380]);
T = {P1, I1, D1};  X = {Kp_list, Ki_list, 1e3*Kd_list};  sel = [Kp_sel Ki_sel 1e3*Kd_sel];
xl = {'Kp', 'Ki [1/s]', 'Kd [ms]'};  tt = {'1단계 P (Ki = 0, Kd = 0)', ...
    sprintf('2단계 I (Kp = %.2f)', Kp_sel), sprintf('3단계 D (Kp = %.2f, Ki = %.0f)', Kp_sel, Ki_sel)};
for k = 1:3
    subplot(1,3,k); yyaxis left;  plot(X{k}, T{k}.itae, 'o-'); ylabel('ITAE / 외란');
    yyaxis right; plot(X{k}, T{k}.os_pct, 's--'); ylabel('오버슈트 [%]');
    xline(sel(k), 'k--'); grid on; xlabel(xl{k}); title(tt{k});
end

%% local functions
function T = run_stage(mdl, Kp, Ki, Kd, which, seeds, r_small, band, J, step_time, Ts, Nn, nzx, nzy)
% 한 게인만 바꿔 가며 seed별로 시뮬레이션 (Simulink가 읽는 base 워크스페이스에 씀)
    switch which
        case 'Kp', vals = Kp;
        case 'Ki', vals = Ki;
        case 'Kd', vals = Kd;
    end
    n = numel(vals);  nS = numel(seeds);
    IT = NaN(n, nS);  ST = NaN(n, nS);  OS = NaN(n, nS);  JT = NaN(n, nS);
    tn = (0:Nn-1).' * Ts;
    for iv = 1:n
        kp = Kp(1); ki = Ki(1); kd = Kd(1);
        switch which
            case 'Kp', kp = vals(iv);
            case 'Ki', ki = vals(iv);
            case 'Kd', kd = vals(iv);
        end
        assignin('base', 'Kp_x', kp/J(1,1));  assignin('base', 'Kp_y', kp/J(2,2));
        assignin('base', 'Ki_x', ki/J(1,1));  assignin('base', 'Ki_y', ki/J(2,2));
        assignin('base', 'Kd_x', kd/J(1,1));  assignin('base', 'Kd_y', kd/J(2,2));
        for is = 1:nS
            rs = RandStream('mt19937ar', 'Seed', seeds(is));
            r  = r_small(1) + diff(r_small)*rand(rs);
            th = 2*pi*rand(rs);
            dvec = r*[cos(th); sin(th)];
            du = J \ dvec;
            assignin('base', 'dux', 0);  assignin('base', 'duy', 0);
            assignin('base', 'dux_step', du(1));  assignin('base', 'duy_step', du(2));
            i0 = 50*(is-1);                                   % seed마다 다른 구간의 잡음
            assignin('base', 'noise_x', [tn nzx(i0+(1:Nn))]);
            assignin('base', 'noise_y', [tn nzy(i0+(1:Nn))]);

            out = sim(mdl);
            [t, Xs] = get_log(out, 'log_X');
            [~, Ys] = get_log(out, 'log_Y');
            [IT(iv,is), ST(iv,is), OS(iv,is), JT(iv,is)] = metric(t - step_time, [Xs Ys], dvec, band, ki > 0);
        end
        fprintf('  %s = %g 완료\n', which, vals(iv));
    end
    T = table(vals(:), mean(IT, 2), mean(ST, 2, 'omitnan'), max(OS, [], 2), mean(JT, 2), sum(isnan(ST), 2), ...
        'VariableNames', {which, 'itae', 'settle_ms', 'os_pct', 'jitter_um', 'fail'});
end

function [t, v] = get_log(out, name)
    s = out.get(name);
    if isa(s, 'timeseries'),  t = s.Time;  v = squeeze(s.Data);
    elseif isstruct(s),       t = s.time;  v = squeeze(s.signals.values);
    else,                     t = out.tout; v = s;
    end
    t = t(:);  v = v(:);
end

function [itae, st, os, jt] = metric(t, P, dvec, band, to_zero)
% itae: ∫ t·|e| dt / 외란 (e = 중심까지 거리), 발산하면 Inf
% st  : 정착 시간 [ms] (I 있으면 0까지, 없으면 최종값까지 2 % 밴드)
% os  : 기준점을 지나 반대쪽으로 넘어간 정도 [% of 외란]
% jt  : 마지막 0.5 s 흔들림 rms [um] (평균 위치 기준)
    m = t >= 0;  t = t(m);  P = P(m, :);
    d = norm(dvec);  dh = dvec / d;
    e = sqrt(sum(P.^2, 2));
    if any(~isfinite(e)) || max(e) > 3*d, itae = Inf; st = NaN; os = NaN; jt = NaN; return; end
    itae = trapz(t, t .* e) / d;
    proj  = P * dh;
    final = mean(proj(t >= t(end) - 0.2));
    if to_zero, ref = 0; else, ref = final; end
    [~, ipk] = max(proj);
    os = max(0, ref - min(proj(ipk:end))) / d * 100;
    ed = sqrt(sum((P - ref*dh.').^2, 2));
    last_out = find(ed > band*d, 1, 'last');
    if isempty(last_out),       st = 0;
    elseif last_out < numel(t), st = 1e3 * t(last_out + 1);
    else,                       st = NaN;
    end
    w = t >= t(end) - 0.5;  Pw = P(w, :);
    jt = 1e3 * sqrt(mean(sum((Pw - mean(Pw, 1)).^2, 2)));
end

function [nx, ny, fname] = load_noise(data_dir)
% 최신 NOISE_*.mat의 Raw(dark 제거)를 DoS 중심 기울기로 나눠 위치 잡음 [mm]로 바꿈
%   (중심에서는 네 추정기가 같으므로 DoS 하나로 충분)
    f = dir(fullfile(data_dir, 'NOISE_*.mat'));
    if isempty(f), error('NOISE 파일 없음: noise_3_laser 먼저 실행'); end
    [~, i] = max([f.datenum]);  fname = f(i).name;
    R = load(fullfile(f(i).folder, fname));
    s.XDiff = R.Raw(:,1); s.YDiff = R.Raw(:,2); s.SUM = R.Raw(:,3);
    [~, ~, dx, dy] = estimator_dos(s, 1);
    sl = zeros(1, 2);  ax = 'XY';
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