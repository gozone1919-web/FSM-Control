%% init_pid_sim.m
% PID_laptop.slx 실행 전에 먼저 실행. Simulink가 쓰는 변수를 전부 Base Workspace에 만든다.
% 순서: init_pid_sim -> PID_laptop 실행 (경로는 이 스크립트가 자동 추가)

%% 0. 경로: 프로젝트 루트 (Main 폴더가 있는 곳) 찾아서 path 추가
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
project_root = here;
while ~isfolder(fullfile(project_root, 'Main'))
    parent = fileparts(project_root);
    if strcmp(parent, project_root)
        error('프로젝트 루트(Main 폴더)를 찾지 못함. FSM Control 폴더 안에서 실행할 것');
    end
    project_root = parent;
end
addpath(genpath(project_root));
data_dir = fullfile(project_root, 'Main', 'DATA');

%% 1. 시뮬레이션 / 제어 주기  (E2 실측 2026-11-06)
Ts    = 0.0158;   % [s] 제어 주기 = KPA 새 값 간격 중앙값 (E2)
delay = 0;        % [s] 추가 루프 지연 (새 값 수신 즉시 계산·명령, E2 응답과 일치)
Tstop = 2;        % [s] 시뮬레이션 길이

%% 2. 플랜트
% J = diag(k)·S  (E3_CAL_20261105_134426, DoS 기준)
%   대각: u -> 같은 축 위치 [mm/u] (S3 계산값 40.9 / 37.8과 같음)
%   비대각: 축간 간섭 (uY -> X: -0.72 %, uX -> Y: +1.50 %)
%   부호 +는 E3 조건부 추정값 (실제 방향은 E5 전에 확인)
J = [40.900  -0.296;
      0.567  37.800];   % [mm/u]
u_max = 0.58;     % [u] 명령 제한 = 100 V / 173 V (데이터시트 전압-각도 선형 구간, 그래프 판독)

%% 2-1. FSM 동특성 모델
% 'E4'       : 실측 (명령 -> KPA 응답, 2026-11-06): 지연 16.5 ms + 2차 지연 6.5 ms
% 'datasheet': S46642 데이터시트 기반 (6차 Bessel + 공진)
fsm_source = 'E4';
E4_theta   = 0.0165;          % [s] 지연 (E4_STEP_20261106_120736 피팅)
E4_tau     = 0.0065;          % [s] 2차 지연 시정수

show_plots = false;
fsm_dynamics_model;           % 데이터시트 모델 (비교용으로 항상 계산)
if strcmp(fsm_source, 'E4')
    Ge  = tf(1, conv([E4_tau 1], [E4_tau 1]), 'InputDelay', E4_theta);
    Ged = absorbDelay(c2d(Ge, Ts, 'zoh'));     % 지연을 분자/분모 다항식에 포함
    [nn, dd] = tfdata(Ged, 'v');
    Gdx_num = nn;  Gdx_den = dd;               % X, Y 같은 모델 (E4에서 두 축 차이 작음)
    Gdy_num = nn;  Gdy_den = dd;
    fprintf('FSM 모델: E4 실측 (지연 %.1f ms, 시정수 %.1f ms, Ts %.1f ms)\n', ...
        1e3*E4_theta, 1e3*E4_tau, 1e3*Ts);
else
    fprintf('FSM 모델: 데이터시트\n');
end

%% 3. PID  (E5 첫 실험과 같은 값: 적분만, Ki 20 /s)
Kp_x = 0;  Ki_x = 20/J(1,1);  Kd_x = 0;  N_x = 100;
Kp_y = 0;  Ki_y = 20/J(2,2);  Kd_y = 0;  N_y = 100;

%% 4. 기준값 / 외란
Xref = 0;  Xref_step = 0;     % [mm]
Yref = 0;  Yref_step = 0;     % [mm]
step_time = 0.5;              % [s] 기준값, 외란 스텝 공통 시각

% 4-1. 명령 외란 (주 외란 시험, E5와 같은 방식, PID는 이 값을 모름)
%   원하는 광점 이동 [mm]을 넣으면 커플링까지 고려해 명령 외란 [u]로 환산: d_u = J \ d_mm
%   첫 비교는 바깥 Saturation이 걸리지 않게 작게 잡을 것
dist_random = false;          % 검증용: 실험과 같은 외란 X +0.5 mm
dist_seed   = 1;
dist_r_mm   = [0.2 3.7];      % [mm] 실험(TEST_closed_loop)과 같은 범위   % [수정]

dcmd_mm      = [0; 0];
dcmd_step_mm = [0.5; 0];      % [mm] 실험(E5_TEST)과 같은 외란

if dist_random
    rs  = RandStream('mt19937ar', 'Seed', dist_seed);
    r   = dist_r_mm(1) + diff(dist_r_mm)*rand(rs);   % 크기 [mm]
    th  = 2*pi*rand(rs);                             % 방향 [rad]
    dcmd_step_mm = r*[cos(th); sin(th)];
end
du_base  = J \ dcmd_mm;       % [u]
du_stepv = J \ dcmd_step_mm;  % [u]
dux = du_base(1);  dux_step = du_stepv(1);    % Command Disturbance 블록용
duy = du_base(2);  duy_step = du_stepv(2);

% 4-2. 위치 외란 (보조 시험, 이상적 광점 외란, 기존 Disturbance Injection 블록)
dx   = 0;  dx_step   = 0;     % [mm]
dy   = 0;  dy_step   = 0;     % [mm]

%% 5. Sensor Lookup Table (빔 스캔 실측: Main\DATA\BEAM_SCAN_X_*, BEAM_SCAN_Y_*)
%   δ = α·δ1 + (1-α)·δ2   (δ1: DoS, δ2: LogRatio)
%   각 추정기는 중심 ±fit_half [mm] 직선 피팅으로 기울기 1, 오프셋 0이 되게 정규화
r0_sensor  = 1.65;            % [mm] 빔 스캔 erf 등가 반경 (기록용, LUT 계산에는 안 씀)
est_names  = {'DoS', 'LogRatio', 'Hybrid a=0.9', 'Hybrid a=0.8'};
alpha_list = [1 0 0.9 0.8];   % 1 = DoS, 0 = LogRatio
est_id     = 3;               % 1 = DoS, 2 = LogRatio, 3 = Hybrid 0.9, 4 = Hybrid 0.8
fit_half   = 0.3;             % [mm] 중심 정규화 피팅 범위

[Xs_sensor, Xhat_all, fX, slopeX] = sensor_lut(data_dir, 'X', alpha_list, fit_half);
[Ys_sensor, Yhat_all, fY, slopeY] = sensor_lut(data_dir, 'Y', alpha_list, fit_half);
Xhat_sensor = Xhat_all(est_id, :);   % LUT_X 테이블
Yhat_sensor = Yhat_all(est_id, :);   % LUT_Y 테이블

%% 6. 센서 잡음 (Sensor 안 From Workspace: noise_x, noise_y [mm])
%   LUT 뒤 위치 추정값 X̂/Ŷ에 더해지는 잡음
%   'gauss'   : E3 중심 잡음 σ로 만든 정규분포 잡음 (실측 기록 전 임시)
%   'measured': Main\DATA\NOISE_*.mat (E2에서 루프 주기로 기록한 XDiff, YDiff, SUM)
%               -> 현재 est_id 추정기로 계산한 X̂ 잡음을 그대로 사용 (추정기별 실제 잡음)
%   'off'     : 잡음 없음
noise_mode = 'gauss';
noise_seed = 1;
sigma_xhat = 0.68e-3;         % [mm] E3 중심 잡음 1σ ('gauss'용)
sigma_yhat = 0.80e-3;         % [mm]

Nn = round(Tstop/Ts) + 1;
tn = (0:Nn-1).' * Ts;
switch noise_mode
    case 'gauss'
        rsn = RandStream('mt19937ar', 'Seed', noise_seed);
        nx  = sigma_xhat * randn(rsn, Nn, 1);
        ny  = sigma_yhat * randn(rsn, Nn, 1);
        noise_src = sprintf('gauss (σ = %.2f / %.2f um)', 1e3*sigma_xhat, 1e3*sigma_yhat);
    case 'measured'
        [nx, ny, noise_src] = measured_noise(data_dir, alpha_list(est_id), ...
            slopeX(est_id), slopeY(est_id), Ts, Nn);
    case 'off'
        nx = zeros(Nn, 1);  ny = zeros(Nn, 1);  noise_src = 'off';
    otherwise
        error('noise_mode는 gauss / measured / off 중 하나');
end
noise_x = [tn nx];            % From Workspace 형식 [시간 값]
noise_y = [tn ny];

fprintf('init_pid_sim 완료: Ts = %.4g s, delay = %.4g s, est_id = %d (%s)\n', ...
    Ts, delay, est_id, est_names{est_id});
fprintf('  Sensor LUT  X: %s (±%.2f mm), Y: %s (±%.2f mm)\n', ...
    fX, max(abs(Xs_sensor)), fY, max(abs(Ys_sensor)));
fprintf('  명령 외란 (seed %d): 광점 [%+.3f %+.3f] mm -> du [%+.5f %+.5f] u\n', ...
    dist_seed, dcmd_step_mm(1), dcmd_step_mm(2), dux_step, duy_step);
fprintf('  센서 잡음: %s, 실제 표준편차 X %.2f um, Y %.2f um\n', ...
    noise_src, 1e3*std(nx), 1e3*std(ny));

%% local functions
function [P, Ph_all, fname, slope] = sensor_lut(data_dir, ax, alpha_list, fit_half)
    f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
    if isempty(f), error('빔 스캔 파일 없음 (%s축): %s', ax, data_dir); end
    [~, i] = max([f.datenum]);
    fname = f(i).name;
    B = load(fullfile(f(i).folder, fname));

    [P, idx]  = sort(B.X(:));        % 스캔 축 실제 위치 [mm] = J * u
    sig.XDiff = B.M(idx,1);          % dark 뺀 평균값
    sig.YDiff = B.M(idx,2);
    sig.SUM   = B.M(idx,3);

    [~, ~, d1x, d1y] = estimator_dos(sig, 1);
    [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
    if strcmp(ax, 'X'), d1 = d1x; d2 = d2x; else, d1 = d1y; d2 = d2y; end
    if any(isnan([d1; d2])), error('%s축 빔 스캔에 무효 점(NaN)이 있음', ax); end

    Ph_all = zeros(numel(alpha_list), numel(P));
    slope  = zeros(numel(alpha_list), 1);
    m = abs(P) <= fit_half;
    for j = 1:numel(alpha_list)
        a = alpha_list(j);
        d = a*d1 + (1 - a)*d2;
        p = polyfit(P(m), d(m), 1);  % d ≈ p(1)*P + p(2)
        slope(j) = p(1);             % [1/mm] 중심 기울기 (잡음 환산용)
        Ph_all(j, :) = (d - p(2)).' / p(1);
    end
    P = P.';                         % Lookup Table breakpoints (행 벡터, 증가 순)
end

function [nx, ny, src] = measured_noise(data_dir, a, sx, sy, Ts, Nn)
% NOISE_*.mat: Raw [N x 3] = [XDiff YDiff SUM] (dark 제거), dt [s] = 기록 간격
    f = dir(fullfile(data_dir, 'NOISE_*.mat'));
    if isempty(f), error('실측 잡음 파일(NOISE_*.mat) 없음. E2에서 기록 후 사용'); end
    [~, i] = max([f.datenum]);
    R = load(fullfile(f(i).folder, f(i).name));
    if abs(R.dt - Ts) > 1e-9
        error('잡음 기록 간격 %.4g s와 Ts %.4g s가 다름', R.dt, Ts);
    end
    sig.XDiff = R.Raw(:,1);  sig.YDiff = R.Raw(:,2);  sig.SUM = R.Raw(:,3);
    [~, ~, d1x, d1y] = estimator_dos(sig, 1);
    [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
    dX = a*d1x + (1 - a)*d2x;
    dY = a*d1y + (1 - a)*d2y;
    nx = (dX - mean(dX)) / sx;       % [mm] 평균 뺀 X̂ 잡음
    ny = (dY - mean(dY)) / sy;
    k  = ceil(Nn / numel(nx));       % 기록이 짧으면 반복
    nx = repmat(nx, k, 1);  nx = nx(1:Nn);
    ny = repmat(ny, k, 1);  ny = ny(1:Nn);
    src = sprintf('measured (%s)', f(i).name);
end