%% tune_fine.m
% Fine tracking 기준 PI 게인 선정 (시뮬레이션 전용, 기존 파일 수정 없음)
%   외란: ESA SILEX 플랫폼 진동 PSD 모양 S(f) = S0/(1+(f/1Hz)^2), 미러 명령에 더함
%         크기는 QPD 위 RMS 3단계 (3.2 µm = SILEX 15.85 µrad를 L로 그대로 옮긴 값)
%   조건: Ms <= 2,  목표: 진짜 위치 RMS 최소 (정상 상태)
%   센서: 진짜 위치 + 측정 노이즈 (중심 근처라 선형 센서로 둠)
%   게인 단위: 루프 이득 (블록 값 = 값 / J), Kd [s]

init_pid_sim;                              % Ged (E4 이산 모델), Ts, J, data_dir
L_mm   = 202.5;                            % 미러-QPD 거리 (실측 200~205 mm)
amp_um = [3.2 15 50];                      % 외란 RMS (QPD 위, 축당)
Ms_max = 2;
Kp_g = 0.1:0.1:1.0;  Ki_g = [5 10 15 20 25 30 40 50 60];
Kd_g = [0 0.0005 0.001 0.002 0.004];  N_d = 30;
seeds = 1:5;  T_run = 30;  T_warm = 2;     % [s] 앞 2 s는 버림
f_c = 1;                                   % [Hz] SILEX 꺾임 주파수

th = 0.0194 + Ts/2;  tau = 0.0106;         % SIMC (E4 FOPDT)
Kc = tau/(2*th);  ref = struct('name', {'기준 PID', 'SIMC'}, 'Kp', {0.5, Kc}, 'Ki', {20, Kc/min(tau, 8*th)});

%% 1. 외란, 노이즈 (seed별로 고정해서 모든 게인에 같은 신호 사용)
[nzx, nzy] = load_noise(data_dir);
sn = [std(nzx) std(nzy)];                  % [mm] 측정 위치 노이즈
N  = round((T_run + T_warm)/Ts);  t = (0:N-1).'*Ts;  keep = t >= T_warm;
a  = exp(-2*pi*f_c*Ts);  Nb = round(10/Ts);
D = zeros(N, 2, numel(seeds));  W = D;     % 단위 RMS 외란, 노이즈 [mm]
for s = 1:numel(seeds)
    rs = RandStream('mt19937ar', 'Seed', seeds(s));
    for k = 1:2
        d = filter(sqrt(1 - a^2), [1 -a], randn(rs, N + Nb, 1));
        D(:,k,s) = d(Nb+1:end);
        W(:,k,s) = sn(k)*randn(rs, N, 1);
    end
end
fprintf('측정 노이즈 X %.2f / Y %.2f µm\n', 1e3*sn);

%% 2. Kp × Ki 격자
nA = numel(amp_um);
RMS = NaN(numel(Ki_g), numel(Kp_g), nA);  MS = NaN(numel(Ki_g), numel(Kp_g));
for i = 1:numel(Kp_g)
    for j = 1:numel(Ki_g)
        C = pid(Kp_g(i), Ki_g(j), 0, 0, Ts, 'IFormula', 'BackwardEuler');
        [MS(j,i), m] = run_gain(C, Ged, D, W, t, keep, amp_um);
        RMS(j,i,:) = m(:,1);
    end
end
ok = MS <= Ms_max;

%% 3. 외란 크기별 선정, 기준 PID·SIMC와 비교
sel = zeros(nA, 2);
for q = 1:nA
    R = RMS(:,:,q);  R(~ok) = Inf;
    [~, k] = min(R(:));  [j, i] = ind2sub(size(R), k);
    sel(q,:) = [Kp_g(i) Ki_g(j)];
    if any(i == [1 numel(Kp_g)]) || any(j == [1 numel(Ki_g)])
        fprintf('[경고] 외란 %.1f µm: 선정점이 격자 끝 → 범위 확장 필요\n', amp_um(q));
    end
end
T = table();
for q = 1:nA
    cand = [struct('name', '선정', 'Kp', sel(q,1), 'Ki', sel(q,2)), ref];
    for c = cand
        C = pid(c.Kp, c.Ki, 0, 0, Ts, 'IFormula', 'BackwardEuler');
        [ms, m, gm, pm] = run_gain(C, Ged, D, W, t, keep, amp_um(q));
        T = [T; table(amp_um(q), string(c.name), c.Kp, c.Ki, ms, gm, pm, ...
            1e3*m(1), 1e3*m(1)/L_mm*1e3, 1e3*m(2), 1e3*m(3), 1e3*m(1)/(sqrt(2)*amp_um(q)), ...
            'VariableNames', {'dist_um', 'design', 'Kp', 'Ki', 'Ms', 'GM_dB', 'PM_deg', ...
            'RMS_um', 'RMS_urad', 'sigma_um', 'bias_um', 'RMS_ratio'})]; %#ok<AGROW>
    end
end
fprintf('\n=== 정상 상태 진짜 위치 오차 (반경 RMS, seed %d개 평균, %d s) ===\n', numel(seeds), T_run);
fprintf('RMS_ratio = 제어 후 반경 RMS / 외란만 있을 때 반경 RMS (1보다 작을수록 잘 누름)\n');
disp(T)
same = all(sel == sel(1,:), 'all');
fprintf('외란 크기별 선정 게인: '); fprintf('(%.1f, %g) ', sel.');
fprintf('→ %s\n', ifelse(same, '세 단계 모두 같음', '외란 크기에 따라 다름'));

%% 4. D항: 기준 외란(15 µm) 선정 PI에 Kd 추가
qs = find(amp_um == 15, 1);  P = sel(qs,:);
KD = NaN(numel(Kd_g), 2 + nA);
for k = 1:numel(Kd_g)
    C = pid(P(1), P(2), Kd_g(k), 1/N_d, Ts, 'IFormula', 'BackwardEuler', 'DFormula', 'BackwardEuler');
    [ms, m] = run_gain(C, Ged, D, W, t, keep, amp_um);
    KD(k,:) = [Kd_g(k) ms 1e3*m(:,1).'];
end
fprintf('\n=== Kd 추가 (Kp %.1f, Ki %g, 미분 필터 N = %d) ===\n', P, N_d);
disp(array2table(KD, 'VariableNames', ["Kd_s", "Ms", replace(compose("RMS_%gum", amp_um), ".", "p")]))

save(fullfile(data_dir, sprintf('TUNE_FINE_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), ...
    'T', 'KD', 'RMS', 'MS', 'sel', 'Kp_g', 'Ki_g', 'Kd_g', 'amp_um', 'L_mm', 'sn', 'seeds', 'T_run');

%% 5. 그림
figure('Name', 'Fine tracking RMS map', 'Color', 'w', 'Position', [80 120 1400 420]);
for q = 1:nA
    subplot(1, nA, q);  hold on;
    R = 1e3*RMS(:,:,q);  R(isnan(MS)) = NaN;
    contourf(Kp_g, Ki_g, R, 20, 'LineColor', 'none');  colorbar;
    contour(Kp_g, Ki_g, MS, [Ms_max Ms_max], 'k', 'LineWidth', 1.5);
    plot(sel(q,1), sel(q,2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    plot(ref(1).Kp, ref(1).Ki, 'wo', 'MarkerSize', 9, 'LineWidth', 1.5);
    plot(ref(2).Kp, ref(2).Ki, 'wd', 'MarkerSize', 9, 'LineWidth', 1.5);
    xlabel('Kp'); ylabel('Ki [1/s]');
    title(sprintf('외란 %.1f µm: RMS [µm]', amp_um(q)));
end
legend({'', 'Ms = 2', '선정', '기준 PID', 'SIMC'}, 'Location', 'southoutside', 'Orientation', 'horizontal');

figure('Name', 'Kd effect', 'Color', 'w');
plot(1e3*Kd_g, KD(:,3:end) ./ KD(1,3:end), '-o', 'LineWidth', 1.2);  grid on;
xlabel('Kd [ms]'); ylabel('RMS / RMS(Kd = 0)');
legend(compose('외란 %.1f µm', amp_um), 'Location', 'best');
title(sprintf('Kd 추가 효과 (Kp %.1f, Ki %g)', P));

%% local functions
function [Ms, m, gm, pm] = run_gain(C, G, D, W, t, keep, amp_um)
% 반환: Ms, m(외란 크기별, [반경 RMS, 반경 σ, |bias|] mm, seed 평균), GM [dB], PM [deg]
    L  = C*G;  S = feedback(1, L);
    nA = numel(amp_um);  m = NaN(nA, 3);  gm = NaN;  pm = NaN;
    if ~isstable(S), Ms = NaN; return; end
    Ms = getPeakGain(S);
    [g, pm] = margin(L);  gm = 20*log10(g);
    Sd = feedback(G, C);  Tn = feedback(L, 1);       % 외란 → 위치, 노이즈 → 위치 (부호 -)
    ns = size(D, 3);  acc = zeros(nA, 3, ns);
    for s = 1:ns
        Yd = [lsim(Sd, D(:,1,s), t) lsim(Sd, D(:,2,s), t)];
        Yn = [lsim(Tn, W(:,1,s), t) lsim(Tn, W(:,2,s), t)];
        for q = 1:nA
            Y = 1e-3*amp_um(q)*Yd(keep,:) - Yn(keep,:);
            b = mean(Y, 1);
            acc(q,:,s) = [sqrt(mean(sum(Y.^2, 2))), sqrt(mean(sum((Y - b).^2, 2))), norm(b)];
        end
    end
    m = mean(acc, 3);
end

function out = ifelse(c, a, b)
    if c, out = a; else, out = b; end
end

function [nx, ny] = load_noise(data_dir)
% 최신 NOISE_*.mat (noise_3_laser)를 DoS 중심 기울기로 위치 잡음 [mm]로 바꿈 (design_pid와 같음)
    f = dir(fullfile(data_dir, 'NOISE_*.mat'));  [~, i] = max([f.datenum]);
    R = load(fullfile(f(i).folder, f(i).name));
    s.XDiff = R.Raw(:,1); s.YDiff = R.Raw(:,2); s.SUM = R.Raw(:,3);
    [~, ~, dx, dy] = estimator_dos(s, 1);
    sl = [0 0];  ax = 'XY';
    for k = 1:2
        g = dir(fullfile(data_dir, ['BEAM_SCAN_' ax(k) '_*.mat']));  [~, j] = max([g.datenum]);
        B = load(fullfile(g(j).folder, g(j).name));  mk = abs(B.X(:)) <= 0.3;
        b.XDiff = B.M(mk,1); b.YDiff = B.M(mk,2); b.SUM = B.M(mk,3);
        [~, ~, ex, ey] = estimator_dos(b, 1);
        if k == 1, p = polyfit(B.X(mk), ex, 1); else, p = polyfit(B.X(mk), ey, 1); end
        sl(k) = p(1);
    end
    nx = (dx - mean(dx)) / sl(1);  ny = (dy - mean(dy)) / sl(2);
end