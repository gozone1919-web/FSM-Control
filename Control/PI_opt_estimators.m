%% PI_opt_estimators.m
% 추정기별 PI 독립 최적화 (시뮬레이션 전용, 하드웨어 사용 안 함)
%   플랜트: E4 실측 모델 (지연 16.5 ms + 2차 지연 6.5 ms), Ts 15.8 ms
%   센서: 최신 빔 스캔(BEAM_SCAN_X/Y) 실측 곡선 + 측정 잡음, 축별 1차원 모델
%   추정기 스케일: v8 (DoS·Hybrid r0, LogRatio k), 영점 보정 유지
%   외란: NEW_TEST와 같은 생성식·seed (계단 0.2~3.7 mm + SILEX 15 µm/축)
%   제약: 루프 이득 기준 Ms <= 2, 폐루프 안정 (추정기 중심 이득 g 반영)
%   최적화 A: 정상 상태 RMSE 최소 / B: 전체 구간 RMSE 최소
%   최적화 seed 1~3, 검증 seed 4~7 (최적화에 안 쓴 seed)

clear; clc;

%% 0. 설정
cfg.Ts      = 0.0158;          % [s] 샘플 주기 (KPA)
cfg.theta   = 0.0165;          % [s] E4 지연
cfg.tau     = 0.0065;          % [s] E4 2차 지연 시정수
cfg.r0_mm   = 1.65;            % [mm] DoS·Hybrid 스케일 (v8)
cfg.k_lr    = 0.43;            % [mm] LogRatio 스케일 (v8, 센서 교정)
cfg.fit_half = 0.3;            % [mm] 영점·중심 기울기 피팅 범위
cfg.T_total = 80;  cfg.t_first = 4;  cfg.dt_ev = [3 4];  cfg.r_mm = [0.2 3.7];
cfg.vib_um  = 15;  cfg.vib_fc = 1;
cfg.u_lim_mm = [4.09 3.78];    % [mm] 명령 한계 (u_lim 0.1 × J)
cfg.band_um = 36;              % [µm] 정착 밴드 하한 (정상 상태 흔들림 약 12 µm × 3)
cfg.Ms_max  = 2;
seeds_opt = 1:3;  seeds_val = 4:7;
Kp_eff = 0.10:0.05:0.80;       % 루프 이득 기준 Kp (= 중심 이득 × 실제 Kp)
Ki_eff = 5:2.5:40;             % 루프 이득 기준 Ki
est_names = {'DoS', 'LogRatio', 'Hybrid 0.9', 'Hybrid 0.8', 'Hybrid 0.5'};
alpha     = [1 0 0.9 0.8 0.5];
L_mm = 202.5;                  % [mm] µrad 환산
make_fig = true;

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

%% 2. 플랜트 (E4, 지연 포함 ZOH 이산화)
Ge = tf(1, conv([cfg.tau 1], [cfg.tau 1]), 'InputDelay', cfg.theta);
Gd = absorbDelay(c2d(Ge, cfg.Ts, 'zoh'));
sysd = ss(Gd);  A = sysd.A;  B = sysd.B;  C = sysd.C;

%% 3. 센서 곡선 (최신 빔 스캔)
SCN = cell(1, 2);  ax_list = {'X', 'Y'};
for ia = 1:2
    f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax_list{ia})));
    [~, i] = max([f.datenum]);  Bs = load(fullfile(f(i).folder, f(i).name));
    [P, idx] = sort(Bs.X(:));
    Z.P = P;  Z.D = Bs.M(idx, ia);  Z.S = Bs.M(idx, 3);  Z.ax = ia;
    [~, i0] = min(abs(Bs.X(:)));
    Z.nD = std(squeeze(Bs.Raw(:, ia, i0)));  Z.nS = std(squeeze(Bs.Raw(:, 3, i0)));
    SCN{ia} = Z;
    fprintf('%s축 빔 스캔: %s (r0 %.3f mm), 잡음 Diff %.3f mV, SUM %.3f mV\n', ...
        ax_list{ia}, f(i).name, Bs.r0, 1e3*Z.nD, 1e3*Z.nS);
end

%% 4. 추정기별 스케일, 영점, 중심 이득
EST = cell(1, numel(alpha));  g = zeros(numel(alpha), 2);
for ie = 1:numel(alpha)
    for ia = 1:2
        Z = SCN{ia};  Z.a = alpha(ie);
        d = mix_delta(Z, Z.D, Z.S);
        m = abs(Z.P) <= cfg.fit_half;  p = polyfit(Z.P(m), d(m), 1);
        Z.b = p(2);  Z.sgn = sign(p(1));
        if alpha(ie) == 0, Z.s = cfg.k_lr; else, Z.s = cfg.r0_mm; end
        g(ie, ia) = Z.s * abs(p(1));
        EST{ie}{ia} = Z;
    end
    fprintf('%-11s 중심 이득 X %.3f, Y %.3f\n', est_names{ie}, g(ie, 1), g(ie, 2));
end

%% 5. 외란 (NEW_TEST와 같은 생성식)
DIST = containers.Map('KeyType', 'double', 'ValueType', 'any');
for s = unique([seeds_opt seeds_val]), DIST(s) = make_dist(s, cfg); end

%% 6. 루프 이득 격자의 Ms (추정기와 무관, g = 1 기준)
MsG = NaN(numel(Ki_eff), numel(Kp_eff));
for i = 1:numel(Kp_eff)
    for j = 1:numel(Ki_eff)
        [ok, ms] = stab(Gd, Kp_eff(i), Ki_eff(j), 1, cfg.Ts);
        if ok, MsG(j, i) = ms; end
    end
end
feas = MsG <= cfg.Ms_max;
fprintf('\nMs <= %.1f 격자점: %d / %d\n', cfg.Ms_max, nnz(feas), numel(feas));

%% 7. 추정기별 최적화 (seed 1~3)
RA = NaN(numel(Ki_eff), numel(Kp_eff), numel(alpha));  RB = RA;
for ie = 1:numel(alpha)
    fprintf('최적화 중: %s ...\n', est_names{ie});
    for i = 1:numel(Kp_eff)
        for j = 1:numel(Ki_eff)
            if ~feas(j, i), continue; end
            Kp = Kp_eff(i)/g(ie, 1);  Ki = Ki_eff(j)/g(ie, 1);
            [a2, b2] = deal(0);
            for s = seeds_opt
                M = metrics(run_sim(EST{ie}, Kp, Ki, DIST(s), A, B, C, cfg, s), DIST(s), cfg);
                a2 = a2 + M.ss_rmse^2;  b2 = b2 + M.total^2;
            end
            RA(j, i, ie) = sqrt(a2/numel(seeds_opt));  RB(j, i, ie) = sqrt(b2/numel(seeds_opt));
        end
    end
end

%% 8. 검증 (seed 4~7): 고정 0.4/20, 중심 이득 보상, 최적 A, 최적 B
T = table();
for ie = 1:numel(alpha)
    ge = g(ie, 1);
    RAe = RA(:,:,ie);  [~, ka] = min(RAe(:));  [ja, ia_] = ind2sub(size(RAe), ka);
    RBe = RB(:,:,ie);  [~, kb] = min(RBe(:));  [jb, ib_] = ind2sub(size(RBe), kb);
    cand = {'고정 0.4/20',     0.4,                 20;
            '중심 이득 보상',   0.4/ge,              20/ge;
            '최적 A (정상)',    Kp_eff(ia_)/ge,      Ki_eff(ja)/ge;
            '최적 B (전체)',    Kp_eff(ib_)/ge,      Ki_eff(jb)/ge};
    base = [];
    for c = 1:size(cand, 1)
        Kp = cand{c, 2};  Ki = cand{c, 3};
        [ok, ms] = stab(Gd, Kp, Ki, ge, cfg.Ts);
        if ~ok
            T = [T; table(string(est_names{ie}), string(cand{c,1}), Kp, Ki, ge*Kp, ge*Ki, NaN, NaN, NaN, NaN, ...
                 NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, "불안정", 'VariableNames', vnames())]; %#ok<AGROW>
            continue;
        end
        Mv = [];
        for s = seeds_val, Mv = [Mv; metrics(run_sim(EST{ie}, Kp, Ki, DIST(s), A, B, C, cfg, s), DIST(s), cfg)]; end %#ok<AGROW>
        ss = sqrt(mean([Mv.ss_rmse].^2));  tot = sqrt(mean([Mv.total].^2));
        dd = vertcat(Mv.dist);  st = vertcat(Mv.st);  os = vertcat(Mv.os);
        b1 = dd < 1;  b2 = dd >= 1 & dd < 2;  b3 = dd >= 2;
        nfail = sum(isnan(st));  nnan = sum([Mv.nan_n]);
        if c == 1, base = [ss tot]; end
        if isempty(base), imp_ss = NaN; imp_tot = NaN; note = "기준(0.4/20) 불안정";
        else, imp_ss = 100*(1 - ss/base(1)); imp_tot = 100*(1 - tot/base(2)); note = ""; end
        T = [T; table(string(est_names{ie}), string(cand{c,1}), Kp, Ki, ge*Kp, ge*Ki, ms, ...
             1e3*ss, 1e3*ss/L_mm*1e3, 1e3*tot, ...
             1e3*median(st(b1),'omitnan'), 1e3*median(st(b2),'omitnan'), 1e3*median(st(b3),'omitnan'), ...
             median(os(b1),'omitnan'), median(os(b2),'omitnan'), median(os(b3),'omitnan'), ...
             nfail, imp_ss, imp_tot, note, 'VariableNames', vnames())]; %#ok<AGROW>
    end
end
fprintf('\n=== 검증 결과 (seed %s, 최적화에 사용 안 함) ===\n', mat2str(seeds_val));
fprintf('Kp_eff, Ki_eff: 루프 이득 기준 (중심 이득 × 실제 게인). RMSE: 정상 상태 µm / µrad, 전체 µm\n');
fprintf('정착 시간 [ms], 오버슈트 [%%]: 외란 0~1 / 1~2 / 2 mm 이상 구간 중앙값. 개선율: 고정 0.4/20 대비 [%%]\n');
disp(T)

out = fullfile(data_dir, sprintf('PI_OPT_SIM_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(out, 'T', 'RA', 'RB', 'MsG', 'Kp_eff', 'Ki_eff', 'g', 'cfg', 'est_names', 'alpha', 'seeds_opt', 'seeds_val');
fprintf('저장: %s\n', out);

%% 9. Kp-Ki 성능 지도 (루프 이득 기준, 검은 선: Ms = 2)
if make_fig
    for k = 1:2
        if k == 1, R = RA; ttl = '정상 상태 RMSE [µm] (최적화 A)'; else, R = RB; ttl = '전체 RMSE [µm] (최적화 B)'; end
        figure('Name', ttl, 'Color', 'w', 'Position', [60 80 1500 330]);
        for ie = 1:numel(alpha)
            subplot(1, numel(alpha), ie); hold on;
            contourf(Kp_eff, Ki_eff, 1e3*R(:,:,ie), 15, 'LineColor', 'none'); colorbar;
            contour(Kp_eff, Ki_eff, MsG, [cfg.Ms_max cfg.Ms_max], 'k', 'LineWidth', 1.5);
            Re = R(:,:,ie);  [~, kk] = min(Re(:));  [jj, ii] = ind2sub(size(Re), kk);
            plot(Kp_eff(ii), Ki_eff(jj), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
            plot(0.4, 20, 'wo', 'MarkerSize', 8, 'LineWidth', 1.5);
            xlabel('Kp_{eff}'); ylabel('Ki_{eff}'); title(est_names{ie});
        end
        sgtitle(ttl);
    end
end

%% local functions
function d = mix_delta(Z, Dv, Sv)
% 축 Z의 Diff, SUM으로 추정기 δ 계산 (기존 estimator 함수 사용)
    if Z.ax == 1, sig.XDiff = Dv; sig.YDiff = zeros(size(Dv)); else, sig.XDiff = zeros(size(Dv)); sig.YDiff = Dv; end
    sig.SUM = Sv;
    [~, ~, d1x, d1y] = estimator_dos(sig, 1);
    [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
    if Z.ax == 1, d1 = d1x; d2 = d2x; else, d1 = d1y; d2 = d2y; end
    if Z.a == 1, d = d1; elseif Z.a == 0, d = d2; else, d = Z.a*d1 + (1 - Z.a)*d2; end
end

function e = sensor_est(Z, x)
% 실제 위치 x [mm] -> 잡음 섞인 Diff, SUM -> 추정 위치 [mm] (v8 스케일, 영점 보정)
    xq = min(max(x, Z.P(1)), Z.P(end));
    Dv = interp1(Z.P, Z.D, xq) + Z.nD*randn;
    Sv = interp1(Z.P, Z.S, xq) + Z.nS*randn;
    e = Z.sgn * Z.s * (mix_delta(Z, Dv, Sv) - Z.b);
end

function D = make_dist(seed, cfg)
% NEW_TEST_closed_loop 3절과 같은 외란
    rs = RandStream('mt19937ar', 'Seed', seed);
    ev_t = [];  ev_d = zeros(2, 0);  tt = cfg.t_first;
    while tt < cfg.T_total - 0.8
        r = cfg.r_mm(1) + diff(cfg.r_mm)*rand(rs);  th = 2*pi*rand(rs);
        ev_t(end+1) = tt;  ev_d(:, end+1) = r*[cos(th); sin(th)];  %#ok<AGROW>
        tt = tt + cfg.dt_ev(1) + diff(cfg.dt_ev)*rand(rs);
    end
    rv = RandStream('mt19937ar', 'Seed', 1000 + seed);
    av = exp(-2*pi*cfg.vib_fc*cfg.Ts);  Nv = ceil(cfg.T_total/cfg.Ts) + 1;  Nb = round(10/cfg.Ts);
    vib = zeros(Nv, 2);
    for k = 1:2
        v = filter(sqrt(1 - av^2), [1 -av], randn(rv, Nv + Nb, 1));
        vib(:, k) = 1e-3*cfg.vib_um * v(Nb+1:end);
    end
    t = (0:Nv-1)'*cfg.Ts;  cum = cumsum(ev_d, 2);  step = zeros(Nv, 2);
    for k = 1:Nv
        j = find(ev_t <= t(k), 1, 'last');
        if ~isempty(j), step(k, :) = cum(:, j).'; end
    end
    D.t = t;  D.ev_t = ev_t;  D.ev_d = ev_d;  D.step = step;  D.vib = vib;
end

function R = run_sim(Ez, Kp, Ki, D, A, B, C, cfg, seed)
% 축별 폐루프 시뮬레이션. R.P: 실제 위치 [mm], R.nan_n: 추정 불가 샘플 수
    rng(10000 + seed);
    N = numel(D.t);  nx = size(A, 1);  x = zeros(nx, 2);
    integ = [0 0];  est_c = [0 0];  P = NaN(N, 2);  nan_n = 0;  lim = cfg.u_lim_mm;
    for k = 1:N
        y = [C*x(:,1), C*x(:,2)];  P(k, :) = y;
        if any(abs(y) > 20), break; end                     % 발산
        for a = 1:2
            e = sensor_est(Ez{a}, y(a));
            if isfinite(e), est_c(a) = e; else, nan_n = nan_n + 1; end
        end
        err = -est_c;  integ = integ + Ki*cfg.Ts*err;  u = integ + Kp*err;
        u = u + D.step(k, :) + D.vib(k, :);
        u = max(min(u, lim), -lim);
        for a = 1:2, x(:, a) = A*x(:, a) + B*u(a); end
    end
    R.P = P;  R.nan_n = nan_n;
end

function M = metrics(R, D, cfg)
% 정상 상태 RMSE(각 외란 직전 1.5 s), 전체 RMSE(첫 외란 이후), 이벤트별 정착 시간·오버슈트
    P = R.P;  t = D.t;  ev = D.ev_t;  ed = D.ev_d;
    if any(isnan(P(:))), M = struct('ss_rmse', Inf, 'total', Inf, 'dist', NaN, 'st', NaN, 'os', NaN, 'nan_n', R.nan_n); return; end
    r = sqrt(sum(P.^2, 2));
    M.total = sqrt(mean(r(t >= ev(1)).^2));
    ss = zeros(0, 2);  ne = numel(ev);  M.dist = NaN(ne, 1);  M.st = NaN(ne, 1);  M.os = NaN(ne, 1);
    for j = 1:ne
        te = t(end);  if j < ne, te = ev(j+1); end
        w = t >= ev(j) & t < te;  ws = t >= te - 1.5 & t < te - 0.05;
        ss = [ss; P(ws, :)]; %#ok<AGROW>
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
    end
    M.ss_rmse = sqrt(mean(sum(ss.^2, 2)));  M.nan_n = R.nan_n;
end

function [ok, Ms] = stab(Gd, Kp, Ki, g, Ts)
% 중심 선형 모델 기준 폐루프 안정 여부, 최대 감도
    Cz = pid(Kp, Ki, 0, 0, Ts, 'IFormula', 'BackwardEuler');
    S = feedback(1, g*Cz*Gd);
    ok = isstable(S);  Ms = NaN;
    if ok, Ms = getPeakGain(S); end
end

function v = vnames()
    v = {'estimator', 'design', 'Kp', 'Ki', 'Kp_eff', 'Ki_eff', 'Ms', 'ss_um', 'ss_urad', 'total_um', ...
         'st_0_1', 'st_1_2', 'st_2_', 'os_0_1', 'os_1_2', 'os_2_', 'no_settle', 'imp_ss_pct', 'imp_tot_pct', 'note'};
end