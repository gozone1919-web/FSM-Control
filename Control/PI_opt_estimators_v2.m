%% PI_opt_estimators_v2.m
% 추정기별 PI 독립 최적화 v2 (시뮬레이션 전용, 하드웨어 사용 안 함)
%   v1 대비: Ms 두 축 검사, A는 SILEX만, 하드웨어 재포착 로직, NaN·coarse 기록,
%            최적점 주변 세밀 탐색, 추정기별 중간 저장, 센서 계산 직접 구현(속도)
%   플랜트: E4 (지연 16.5 ms + 2차 6.5 ms), Ts 15.8 ms
%   센서: 최신 빔 스캔 실측 곡선 + 측정 잡음, 축별 1차원 (2차원 광량 = SUM_X·SUM_Y / SUM0)
%   추정기 스케일: v8 (DoS·Hybrid r0, LogRatio k), 영점 보정 유지
%   최적화 seed 1~3, 검증 seed 4~7

clear; clc;

%% 0. 설정
cfg.Ts = 0.0158;  cfg.theta = 0.0165;  cfg.tau = 0.0065;
cfg.r0_mm = 1.65;  cfg.k_lr = 0.43;  cfg.fit_half = 0.3;
cfg.T_total = 80;  cfg.t_first = 4;  cfg.dt_ev = [3 4];  cfg.r_mm = [0.2 3.7];
cfg.vib_um = 15;  cfg.vib_fc = 1;
cfg.T_vib = 40;  cfg.t_skip = 5;          % 최적화 A: SILEX만 40 s, 앞 5 s 버림
cfg.u_lim_mm = [4.09 3.78];
cfg.loss_ratio = 0.93;  cfg.loss_hold = 0.2;  cfg.nan_hold = 0.5;   % 하드웨어와 같음
cfg.band_um = 36;  cfg.Ms_max = 2;
seeds_opt = 1:3;  seeds_val = 4:7;
Kp_eff = 0.10:0.05:0.80;  Ki_eff = 5:2.5:40;          % 1차 격자 (루프 이득 기준, X축 중심 이득)
fine_dKp = -0.04:0.01:0.04;  fine_dKi = -2:0.5:2;     % 2차 세밀 탐색
est_names = {'DoS', 'LogRatio', 'Hybrid 0.9', 'Hybrid 0.8', 'Hybrid 0.5'};
alpha = [1 0 0.9 0.8 0.5];
L_mm = 202.5;  make_fig = true;

%% 1. 경로
here = fileparts(mfilename('fullpath'));  if isempty(here), here = pwd; end
root = here;
while ~isfolder(fullfile(root, 'Main'))
    parent = fileparts(root);
    if strcmp(parent, root), error('프로젝트 루트(Main 폴더)를 찾지 못함'); end
    root = parent;
end
addpath(genpath(root));
data_dir = fullfile(root, 'Main', 'DATA');
stamp = datestr(now, 'yyyymmdd_HHMMSS');

%% 2. 플랜트
Ge = tf(1, conv([cfg.tau 1], [cfg.tau 1]), 'InputDelay', cfg.theta);
Gd = absorbDelay(c2d(Ge, cfg.Ts, 'zoh'));
sysd = ss(Gd);  A = sysd.A;  B = sysd.B;  C = sysd.C;

%% 3. 센서 곡선
SCN = cell(1, 2);  ax_list = {'X', 'Y'};
for ia = 1:2
    f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax_list{ia})));
    [~, i] = max([f.datenum]);  Bs = load(fullfile(f(i).folder, f(i).name));
    [P, idx] = sort(Bs.X(:));
    Z.P = P;  Z.dP = mean(diff(P));  Z.D = Bs.M(idx, ia);  Z.S = Bs.M(idx, 3);  Z.ax = ia;
    [~, i0] = min(abs(Bs.X(:)));
    Z.nD = std(squeeze(Bs.Raw(:, ia, i0)));  Z.nS = std(squeeze(Bs.Raw(:, 3, i0)));
    SCN{ia} = Z;
    fprintf('%s축 빔 스캔: %s (r0 %.3f mm), 잡음 Diff %.3f mV, SUM %.3f mV\n', ...
        ax_list{ia}, f(i).name, Bs.r0, 1e3*Z.nD, 1e3*Z.nS);
end
S0 = interp1(SCN{1}.P, SCN{1}.S, 0);       % 중심 SUM (광량 손실 기준)

%% 4. 추정기별 스케일, 영점, 중심 이득
EST = cell(1, numel(alpha));  g = zeros(numel(alpha), 2);
for ie = 1:numel(alpha)
    for ia = 1:2
        Z = SCN{ia};  Z.a = alpha(ie);
        d = mix_delta(Z, Z.D, Z.S);
        m = abs(Z.P) <= cfg.fit_half;  p = polyfit(Z.P(m), d(m), 1);
        Z.b = p(2);  Z.sgn = sign(p(1));
        if alpha(ie) == 0, Z.s = cfg.k_lr; else, Z.s = cfg.r0_mm; end
        g(ie, ia) = Z.s*abs(p(1));
        EST{ie}{ia} = Z;
    end
    fprintf('%-11s 중심 이득 X %.3f, Y %.3f\n', est_names{ie}, g(ie, 1), g(ie, 2));
end

%% 5. 외란 (NEW_TEST와 같은 생성식)
DB = containers.Map('KeyType', 'double', 'ValueType', 'any');   % 계단 + SILEX
DA = containers.Map('KeyType', 'double', 'ValueType', 'any');   % SILEX만
for s = unique([seeds_opt seeds_val])
    DB(s) = make_dist(s, cfg, true,  cfg.T_total);
    DA(s) = make_dist(s, cfg, false, cfg.T_vib);
end

%% 6. 추정기별 최적화
nP = numel(Kp_eff);  nI = numel(Ki_eff);
RA = NaN(nI, nP, numel(alpha));  RB = RA;  MSM = RA;
OPT = struct('est', {}, 'A', {}, 'B', {});
for ie = 1:numel(alpha)
    t0 = tic;  gx = g(ie, 1);  gy = g(ie, 2);
    fprintf('\n최적화: %s (중심 이득 X %.3f, Y %.3f)\n', est_names{ie}, gx, gy);
    for i = 1:nP
        for j = 1:nI
            msm = ms_both(Gd, Kp_eff(i), Ki_eff(j), gy/gx, cfg.Ts);
            MSM(j, i, ie) = msm;
            if ~(msm <= cfg.Ms_max), continue; end
            [RA(j, i, ie), RB(j, i, ie)] = objective(EST{ie}, Kp_eff(i)/gx, Ki_eff(j)/gx, seeds_opt, DA, DB, A, B, C, cfg, S0);
        end
    end
    best = struct();
    for obj = 'AB'
        if obj == 'A', R = RA(:,:,ie); else, R = RB(:,:,ie); end
        if all(~isfinite(R(:))), fprintf('  [%s] 유효한 격자점 없음\n', obj); best.(obj) = []; continue; end
        [~, k] = min(R(:));  [j0, i0] = ind2sub(size(R), k);
        if any(i0 == [1 nP]) || any(j0 == [1 nI])
            fprintf('  [경고] 최적화 %s: 1차 최적점이 격자 끝 (Kp_eff %.2f, Ki_eff %.1f) → 범위 확장 필요\n', obj, Kp_eff(i0), Ki_eff(j0));
        end
        bp = [Kp_eff(i0) Ki_eff(j0) R(k)];
        for dkp = fine_dKp                                   % 2차 세밀 탐색
            for dki = fine_dKi
                kp = Kp_eff(i0) + dkp;  ki = Ki_eff(j0) + dki;
                if kp <= 0 || ki <= 0 || (dkp == 0 && dki == 0), continue; end
                if ~(ms_both(Gd, kp, ki, gy/gx, cfg.Ts) <= cfg.Ms_max), continue; end
                [ra, rb] = objective(EST{ie}, kp/gx, ki/gx, seeds_opt, DA, DB, A, B, C, cfg, S0);
                if obj == 'A', r = ra; else, r = rb; end
                if r < bp(3), bp = [kp ki r]; end
            end
        end
        best.(obj) = bp;
        fprintf('  최적 %s: Kp_eff %.3f, Ki_eff %.2f (실제 Kp %.4f, Ki %.3f), 목적함수 %.2f µm\n', ...
            obj, bp(1), bp(2), bp(1)/gx, bp(2)/gx, 1e3*bp(3));
    end
    OPT(ie).est = est_names{ie};  OPT(ie).A = best.A;  OPT(ie).B = best.B;
    save(fullfile(data_dir, sprintf('PI_OPT_SIM2_partial_%s.mat', stamp)), ...
        'RA', 'RB', 'MSM', 'OPT', 'Kp_eff', 'Ki_eff', 'g', 'cfg', 'est_names', 'alpha');
    fprintf('  %.0f s, 중간 저장\n', toc(t0));
end

%% 7. 검증 (seed 4~7)
T = table();
for ie = 1:numel(alpha)
    gx = g(ie, 1);  gy = g(ie, 2);
    cand = {'고정 0.4/20', 0.4, 20;  '중심 이득 보상', 0.4/gx, 20/gx};
    if ~isempty(OPT(ie).A), cand(end+1, :) = {'최적 A (정상)', OPT(ie).A(1)/gx, OPT(ie).A(2)/gx}; end
    if ~isempty(OPT(ie).B), cand(end+1, :) = {'최적 B (전체)', OPT(ie).B(1)/gx, OPT(ie).B(2)/gx}; end
    base = [];
    for c = 1:size(cand, 1)
        Kp = cand{c, 2};  Ki = cand{c, 3};
        [okx, msx] = stab(Gd, Kp, Ki, gx, cfg.Ts);  [oky, msy] = stab(Gd, Kp, Ki, gy, cfg.Ts);
        if ~(okx && oky)
            T = [T; row(est_names{ie}, cand{c,1}, Kp, Ki, gx, msx, msy, NaN(1, 15), "불안정")]; %#ok<AGROW>
            continue;
        end
        ssv = [];  MB = [];
        for s = seeds_val
            RAv = run_sim(EST{ie}, Kp, Ki, DA(s), A, B, C, cfg, s, S0);  ssv(end+1) = metric_A(RAv, DA(s), cfg); %#ok<AGROW>
            MB = [MB; metric_B(run_sim(EST{ie}, Kp, Ki, DB(s), A, B, C, cfg, s, S0), DB(s), cfg)]; %#ok<AGROW>
        end
        ss = sqrt(mean(ssv.^2));  tot = sqrt(mean([MB.total].^2));
        dd = vertcat(MB.dist);  st = vertcat(MB.st);  os = vertcat(MB.os);  cz = vertcat(MB.ev_coarse);  en = vertcat(MB.ev_nan);
        st(cz) = NaN;  os(cz) = NaN;                       % coarse 이벤트는 평균에서 제외 (하드웨어와 같음)
        b1 = dd < 1;  b2 = dd >= 1 & dd < 2;  b3 = dd >= 2;
        vals = [1e3*ss, 1e3*ss/L_mm*1e3, 1e3*tot, ...
                1e3*median(st(b1),'omitnan'), 1e3*median(st(b2),'omitnan'), 1e3*median(st(b3),'omitnan'), ...
                median(os(b1),'omitnan'), median(os(b2),'omitnan'), median(os(b3),'omitnan'), ...
                sum(isnan(st) & ~cz), sum([MB.nan_n]), 100*mean(en), sum(cz), NaN, NaN];
        if c == 1, base = [ss tot]; end
        note = "";
        if isempty(base), note = "기준(0.4/20) 불안정";
        else, vals(14) = 100*(1 - ss/base(1));  vals(15) = 100*(1 - tot/base(2)); end
        T = [T; row(est_names{ie}, cand{c,1}, Kp, Ki, gx, msx, msy, vals, note)]; %#ok<AGROW>
    end
end
fprintf('\n=== 검증 결과 (seed %s, 최적화에 사용 안 함) ===\n', mat2str(seeds_val));
fprintf('ss: SILEX만 정상 상태 RMSE [µm, µrad] / total: 계단 + SILEX 전체 RMSE [µm]\n');
fprintf('st, os: 외란 0~1 / 1~2 / 2 mm 이상 중앙값 (coarse 제외). no_settle: 정착 실패, nan_ev_pct: NaN 발생 이벤트 비율\n');
fprintf('imp: 고정 0.4/20 대비 개선율 [%%]. Kp_eff, Ki_eff: X축 중심 이득 기준\n');
disp(T)
out = fullfile(data_dir, sprintf('PI_OPT_SIM2_%s.mat', stamp));
save(out, 'T', 'RA', 'RB', 'MSM', 'OPT', 'Kp_eff', 'Ki_eff', 'g', 'cfg', 'est_names', 'alpha', 'seeds_opt', 'seeds_val');
fprintf('저장: %s\n', out);

%% 8. 성능 지도 (검은 선: 두 축 중 큰 Ms = 2)
if make_fig
    for k = 1:2
        if k == 1, R = RA; ttl = '정상 상태 RMSE [µm] (최적화 A, SILEX만)'; else, R = RB; ttl = '전체 RMSE [µm] (최적화 B)'; end
        figure('Name', ttl, 'Color', 'w', 'Position', [60 80 1500 330]);
        for ie = 1:numel(alpha)
            subplot(1, numel(alpha), ie); hold on;
            contourf(Kp_eff, Ki_eff, 1e3*R(:,:,ie), 15, 'LineColor', 'none'); colorbar;
            contour(Kp_eff, Ki_eff, MSM(:,:,ie), [cfg.Ms_max cfg.Ms_max], 'k', 'LineWidth', 1.5);
            if k == 1, bp = OPT(ie).A; else, bp = OPT(ie).B; end
            if ~isempty(bp), plot(bp(1), bp(2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r'); end
            plot(0.4*g(ie,1), 20*g(ie,1), 'wo', 'MarkerSize', 8, 'LineWidth', 1.5);   % 고정 0.4/20 위치
            xlabel('Kp_{eff}'); ylabel('Ki_{eff}'); title(est_names{ie});
        end
        sgtitle(ttl);
    end
end

%% local functions
function [d, ok] = mix_delta(Z, Dv, Sv)
% estimator_dos / estimator_logratio와 같은 식 (직접 계산)
%   X: δD = -XDiff/SUM, δL = log(Rt/L), L = (SUM+XDiff)/2, Rt = (SUM-XDiff)/2
%   Y: δD =  YDiff/SUM, δL = log(T/B),  T = (SUM+YDiff)/2, B  = (SUM-YDiff)/2
    hp = (Sv + Dv)/2;  hm = (Sv - Dv)/2;  ok = hp > 0 & hm > 0;
    if Z.ax == 1, d1 = -Dv./Sv;  d2 = log(hm./hp); else, d1 = Dv./Sv;  d2 = log(hp./hm); end
    d2(~ok) = NaN;
    if Z.a == 1, d = d1; elseif Z.a == 0, d = d2; else, d = Z.a*d1 + (1 - Z.a)*d2; end
end

function [e, Sv] = sensor_est(Z, x)
    q = (min(max(x, Z.P(1)), Z.P(end)) - Z.P(1))/Z.dP + 1;
    i = min(max(floor(q), 1), numel(Z.P) - 1);  w = q - i;
    Dv = Z.D(i)*(1 - w) + Z.D(i+1)*w + Z.nD*randn;
    Sv = Z.S(i)*(1 - w) + Z.S(i+1)*w + Z.nS*randn;
    e = Z.sgn*Z.s*(mix_delta(Z, Dv, Sv) - Z.b);
end

function D = make_dist(seed, cfg, with_steps, T)
    rs = RandStream('mt19937ar', 'Seed', seed);
    ev_t = [];  ev_d = zeros(2, 0);  tt = cfg.t_first;
    while tt < cfg.T_total - 0.8
        r = cfg.r_mm(1) + diff(cfg.r_mm)*rand(rs);  th = 2*pi*rand(rs);
        ev_t(end+1) = tt;  ev_d(:, end+1) = r*[cos(th); sin(th)];  %#ok<AGROW>
        tt = tt + cfg.dt_ev(1) + diff(cfg.dt_ev)*rand(rs);
    end
    rv = RandStream('mt19937ar', 'Seed', 1000 + seed);
    av = exp(-2*pi*cfg.vib_fc*cfg.Ts);  Nv0 = ceil(cfg.T_total/cfg.Ts) + 1;  Nb = round(10/cfg.Ts);
    vib = zeros(Nv0, 2);
    for k = 1:2
        v = filter(sqrt(1 - av^2), [1 -av], randn(rv, Nv0 + Nb, 1));
        vib(:, k) = 1e-3*cfg.vib_um*v(Nb+1:end);
    end
    Nv = ceil(T/cfg.Ts) + 1;  t = (0:Nv-1)'*cfg.Ts;  vib = vib(1:Nv, :);
    if ~with_steps, ev_t = [];  ev_d = zeros(2, 0); end
    step = zeros(Nv, 2);
    if ~isempty(ev_t)
        cum = cumsum(ev_d, 2);
        for k = 1:Nv
            j = find(ev_t <= t(k), 1, 'last');
            if ~isempty(j), step(k, :) = cum(:, j).'; end
        end
    end
    D.t = t;  D.ev_t = ev_t;  D.ev_d = ev_d;  D.step = step;  D.vib = vib;
end

function R = run_sim(Ez, Kp, Ki, D, A, B, C, cfg, seed, S0)
% 하드웨어 run_loop와 같은 NaN 유지·재포착 로직
    rng(10000 + seed);
    N = numel(D.t);  nx = size(A, 1);  x = zeros(nx, 2);  Ts = cfg.Ts;  lim = cfg.u_lim_mm;
    integ = [0 0];  est_c = [0 0];  P = NaN(N, 2);  nanf = false(N, 1);  coarse = zeros(N, 1);
    t_loss0 = NaN;  t_nan0 = NaN;  reacq = false;
    for k = 1:N
        t = D.t(k);
        y = [C*x(:,1), C*x(:,2)];  P(k, :) = y;
        if any(abs(y) > 20), break; end
        [e1, S1] = sensor_est(Ez{1}, y(1));  [e2, S2] = sensor_est(Ez{2}, y(2));
        e = [e1 e2];  Stot = S1*S2/S0;
        isn = any(~isfinite(e));  lossS = Stot < cfg.loss_ratio*S0;  nanf(k) = isn;
        if isn,   if isnan(t_nan0),  t_nan0  = t; end, else, t_nan0  = NaN; end
        if lossS, if isnan(t_loss0), t_loss0 = t; end, else, t_loss0 = NaN; end
        if ~isn, est_c = e; end
        if reacq && ~isn && ~lossS, reacq = false; end
        if ~reacq && ((~isnan(t_loss0) && t - t_loss0 >= cfg.loss_hold) || (~isnan(t_nan0) && t - t_nan0 >= cfg.nan_hold))
            if lossS, coarse(k) = 1; else, coarse(k) = 2; end
            integ = -D.step(k, :);  upid = integ;  est_c = [0 0];  reacq = true;  t_loss0 = NaN;  t_nan0 = NaN;
        else
            err = -est_c;  integ = integ + Ki*Ts*err;  upid = integ + Kp*err;
        end
        u = max(min(upid + D.step(k, :) + D.vib(k, :), lim), -lim);
        for a = 1:2, x(:, a) = A*x(:, a) + B*u(a); end
    end
    R.P = P;  R.nanf = nanf;  R.coarse = coarse;
end

function [ra, rb] = objective(Ez, Kp, Ki, seeds, DA, DB, A, B, C, cfg, S0)
    a2 = 0;  b2 = 0;
    for s = seeds
        a2 = a2 + metric_A(run_sim(Ez, Kp, Ki, DA(s), A, B, C, cfg, s, S0), DA(s), cfg)^2;
        M = metric_B(run_sim(Ez, Kp, Ki, DB(s), A, B, C, cfg, s, S0), DB(s), cfg);
        b2 = b2 + M.total^2;
    end
    ra = sqrt(a2/numel(seeds));  rb = sqrt(b2/numel(seeds));
end

function r = metric_A(R, D, cfg)
% SILEX만: 앞 t_skip 이후 반경 RMSE
    if any(isnan(R.P(:))), r = Inf; return; end
    w = D.t >= cfg.t_skip;  r = sqrt(mean(sum(R.P(w, :).^2, 2)));
end

function M = metric_B(R, D, cfg)
% 계단 + SILEX: 전체 RMSE, 이벤트별 정착 시간·오버슈트·NaN·coarse
    P = R.P;  t = D.t;  ev = D.ev_t;  ed = D.ev_d;  ne = numel(ev);
    M.dist = NaN(ne, 1);  M.st = NaN(ne, 1);  M.os = NaN(ne, 1);  M.ev_nan = false(ne, 1);  M.ev_coarse = false(ne, 1);
    M.nan_n = sum(R.nanf);
    if any(isnan(P(:))), M.total = Inf; return; end
    r = sqrt(sum(P.^2, 2));  M.total = sqrt(mean(r(t >= ev(1)).^2));
    for j = 1:ne
        te = t(end);  if j < ne, te = ev(j+1); end
        w = t >= ev(j) & t < te;
        d = norm(ed(:, j));  dh = ed(:, j)/d;  tj = t(w) - ev(j);  e = r(w);
        out = e > max(0.02*d, 1e-3*cfg.band_um);
        last = 0;  run = 0;
        for i = 1:numel(out)
            if out(i), run = run + 1; else, run = 0; end
            if run >= 3 || (out(i) && all(out(1:i))), last = i; end
        end
        if last == 0, st = 0;
        else
            i2 = last;  while i2 < numel(out) && out(i2+1), i2 = i2 + 1; end
            if i2 < numel(tj), st = tj(i2+1); else, st = NaN; end
        end
        proj = P(w, :)*dh;  [~, ipk] = max(proj);
        M.dist(j) = d;  M.st(j) = st;  M.os(j) = max(0, -min(proj(ipk:end)))/d*100;
        M.ev_nan(j) = any(R.nanf(w));  M.ev_coarse(j) = any(R.coarse(w) > 0);
    end
end

function [ok, Ms] = stab(Gd, Kp, Ki, g, Ts)
    Cz = pid(Kp, Ki, 0, 0, Ts, 'IFormula', 'BackwardEuler');
    S = feedback(1, g*Cz*Gd);  ok = isstable(S);  Ms = NaN;
    if ok, Ms = getPeakGain(S); end
end

function msm = ms_both(Gd, kp_eff, ki_eff, ry, Ts)
% X축(이득 1)과 Y축(이득 ry = gY/gX) 중 큰 Ms. 불안정이면 Inf
    [okx, msx] = stab(Gd, kp_eff, ki_eff, 1, Ts);  [oky, msy] = stab(Gd, kp_eff, ki_eff, ry, Ts);
    if okx && oky, msm = max(msx, msy); else, msm = Inf; end
end

function r = row(est, design, Kp, Ki, gx, msx, msy, v, note)
    r = table(string(est), string(design), Kp, Ki, gx*Kp, gx*Ki, msx, msy, ...
        v(1), v(2), v(3), v(4), v(5), v(6), v(7), v(8), v(9), v(10), v(11), v(12), v(13), v(14), v(15), string(note), ...
        'VariableNames', {'estimator', 'design', 'Kp', 'Ki', 'Kp_eff', 'Ki_eff', 'Ms_X', 'Ms_Y', ...
        'ss_um', 'ss_urad', 'total_um', 'st_0_1', 'st_1_2', 'st_2_', 'os_0_1', 'os_1_2', 'os_2_', ...
        'no_settle', 'nan_n', 'nan_ev_pct', 'coarse_n', 'imp_ss_pct', 'imp_tot_pct', 'note'});
end