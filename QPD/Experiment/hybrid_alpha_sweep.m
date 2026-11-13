%% hybrid_alpha_sweep.m
% 빔 스캔 실측 데이터로 Hybrid 가중치 α 비교 (X, Y 축) - 레이저 불필요
%   기준 1: 국소 기울기 dX̂/dX 가 1 ± tol 안에 있는 범위 (루프 이득 변화)
%   기준 2: 절대오차 |X̂ - X| 가 thr 이내인 범위 (참고용, 표에만 출력)
clc;

%% 0. 경로
here         = fileparts(mfilename('fullpath'));   % QPD\Experiment
qpd_dir      = fileparts(here);                    % QPD
project_root = fileparts(qpd_dir);                 % 프로젝트 루트
data_dir     = fullfile(project_root, 'Main', 'DATA');
addpath(qpd_dir);                                  % estimator_*.m 함수 경로

%% 1. 설정
alpha_plot  = [0.9 0.85 0.8];   % 그림으로 볼 α
alpha_sweep = 0.5:0.05:1.0;     % 범위 비교용 α
fit_half    = 0.3;              % 중심 기울기 피팅 범위 ±[mm]
slope_tol   = [0.10 0.20];      % 기울기 기준: 1 ± 10 %, 1 ± 20 %
thr_um      = [50 100];         % 절대오차 기준 [um] (참고)
axes_list   = {'X', 'Y'};
RES = struct();

for ia = 1:numel(axes_list)
    ax = axes_list{ia};

    %% 2. 해당 축 최신 빔 스캔 로드
    f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
    if isempty(f) && strcmp(ax, 'X')
        f = dir(fullfile(data_dir, 'BEAM_SCAN_2*.mat'));   % 예전 파일 (X축만 스캔)
    end
    if isempty(f)
        fprintf('\n[%s축] 빔 스캔 파일 없음 -> 건너뜀\n', ax);
        continue;
    end
    [~, i] = max([f.datenum]);
    B = load(fullfile(f(i).folder, f(i).name));
    fprintf('\n[%s축] 빔 스캔: %s\n', ax, f(i).name);

    [P, idx]  = sort(B.X(:));          % 스캔 축 실제 위치 [mm] = J * u
    sig.XDiff = B.M(idx,1);            % dark 뺀 평균값
    sig.YDiff = B.M(idx,2);
    sig.SUM   = B.M(idx,3);

    %% 3. 기본 δ (k = 1), 스캔 축 성분 선택
    [~, ~, d1x, d1y] = estimator_dos(sig, 1);
    [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
    if strcmp(ax, 'X'), d1 = d1x; d2 = d2x; else, d1 = d1y; d2 = d2y; end

    %% 4. α 스윕
    n = numel(alpha_sweep);
    vn = {'alpha', 'k_mm', ...
          sprintf('slope%02d_mm', round(100*slope_tol(1))), ...
          sprintf('slope%02d_mm', round(100*slope_tol(2))), ...
          sprintf('abs%dum_mm', thr_um(1)), sprintf('abs%dum_mm', thr_um(2)), 'sat_mm'};
    R = table('Size', [n numel(vn)], 'VariableTypes', repmat({'double'}, 1, numel(vn)), ...
        'VariableNames', vn);
    for j = 1:n
        a = alpha_sweep(j);
        [Ph, k] = hybrid_xhat(d1, d2, a, P, fit_half);
        R(j, :) = {a, k, ...
                   slope_range(P, Ph, slope_tol(1)), slope_range(P, Ph, slope_tol(2)), ...
                   abs_range(P, Ph, thr_um(1)*1e-3), abs_range(P, Ph, thr_um(2)*1e-3), ...
                   min(max(Ph), -min(Ph))};
    end
    fprintf('slope: 기울기 1±tol 범위, abs: 절대오차 범위, sat: 추정값 최대 (모두 양쪽 중 좁은 쪽, mm)\n');
    disp(R)
    RES.(ax) = R;

    %% 5. 그림 1, 2: α = 0.9 / 0.85 / 0.8 (위치, 오차, 기울기)
    figure('Name', sprintf('Hybrid alpha comparison (%s)', ax), 'Color', 'w', ...
        'Position', [80 80 1400 420]);
    for s = 1:3, subplot(1, 3, s); hold on; grid on; end
    for a = alpha_plot
        Ph  = hybrid_xhat(d1, d2, a, P, fit_half);
        lbl = sprintf('\\alpha = %.2f', a);
        subplot(1, 3, 1); plot(P, Ph, 'o-', 'MarkerSize', 4, 'DisplayName', lbl);
        subplot(1, 3, 2); plot(P, Ph - P, 'o-', 'MarkerSize', 4, 'DisplayName', lbl);
        subplot(1, 3, 3); plot(P, gradient(Ph, P), 'o-', 'MarkerSize', 4, 'DisplayName', lbl);
    end
    subplot(1, 3, 1);
    plot(P, P, 'k--', 'DisplayName', 'y = x');
    xlabel(sprintf('%s true [mm] (J \\times u)', ax)); ylabel(sprintf('%s\\_hat [mm]', ax));
    title(sprintf('%s축: 추정 위치 vs 실제 위치', ax)); legend('Location', 'best');
    subplot(1, 3, 2);
    yline(0, 'k--', 'HandleVisibility', 'off');
    xlabel(sprintf('%s true [mm]', ax)); ylabel('추정 오차 [mm]');
    title(sprintf('%s축: 추정 오차', ax)); legend('Location', 'best');
    subplot(1, 3, 3);
    yline(1, 'k--', 'HandleVisibility', 'off');
    for t = slope_tol
        yline(1 + t, ':', 'HandleVisibility', 'off');
        yline(1 - t, ':', 'HandleVisibility', 'off');
    end
    ylim([-0.2 1.6]);
    xlabel(sprintf('%s true [mm]', ax)); ylabel('국소 기울기 d\_hat / d\_true');
    title(sprintf('%s축: 국소 기울기 (루프 이득 배율)', ax)); legend('Location', 'best');
end

%% 6. 그림 3: α에 따른 기울기 기준 범위 (X, Y)
figure('Name', 'Slope range vs alpha', 'Color', 'w'); hold on; grid on;
mk = {'o-', 's--'};
for ia = 1:numel(axes_list)
    ax = axes_list{ia};
    if ~isfield(RES, ax), continue; end
    R = RES.(ax);
    for it = 1:numel(slope_tol)
        plot(R.alpha, R{:, 2 + it}, mk{it}, ...
            'DisplayName', sprintf('%s축, 기울기 1 \\pm %d%%', ax, round(100*slope_tol(it))));
    end
end
xlabel('\alpha'); ylabel('범위 [mm]');
title('Hybrid 가중치에 따른 범위 (기울기 기준)'); legend('Location', 'best');

%% local functions
function [Ph, k] = hybrid_xhat(d1, d2, a, P, fit_half)
    d  = a*d1 + (1 - a)*d2;
    m  = abs(P) <= fit_half & ~isnan(d);
    p  = polyfit(P(m), d(m), 1);     % d ≈ p(1)*P + p(2)
    k  = 1/p(1);
    Ph = (d - p(2)) * k;
end

function w = slope_range(P, Ph, tol)
    s  = gradient(Ph, P);            % 국소 기울기
    [~, i0] = min(abs(P));
    ok = abs(s - 1) <= tol;
    ip = i0;  while ip < numel(P) && ok(ip+1),  ip = ip + 1;  end
    im = i0;  while im > 1 && ok(im-1),         im = im - 1;  end
    w = min(-P(im), P(ip));
end

function w = abs_range(P, Ph, thr)
    [~, i0] = min(abs(P));
    ok = abs(Ph - P) <= thr;
    ip = i0;  while ip < numel(P) && ok(ip+1),  ip = ip + 1;  end
    im = i0;  while im > 1 && ok(im-1),         im = im - 1;  end
    w = min(-P(im), P(ip));
end