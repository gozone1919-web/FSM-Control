%% NEW_TEST_closed_loop.m
% 실제 빔 폐루프 시험: 랜덤 숨은 스텝 외란 + SILEX 진동, 추정기 여러 개 연속 비교
%   제어: 각 추정기의 X̂ / 평가: 보정된 빔 위치(LogRatio δ -> 빔 스캔 표 역변환)와 정규화 X̂ 두 기준   % [수정 11/27]
%   정상 상태 지표(정착 시점 ~ 다음 외란 전): 반경 RMSE, 편향(평균 위치 크기), 흔들림(평균 뺀 반경 σ), 축별 RMSE   % [수정 11/27]
%     RMSE² = 편향² + 흔들림². µrad 환산은 미러-QPD 202.5 mm 기준   % [수정 11/27]
%   추적 구간: 반대쪽 반 신호가 바닥값의 2배 이상인 범위 (보정된 빔 위치 측정 가능)   % [수정 11/27]
%   탐색 구간: 그 밖 (방향만 앎, 보정된 빔 위치 NaN)   % [수정 11/27]
%   추정값 NaN: PID는 직전 유효값으로 계속 계산, NaN 샘플·상황은 따로 기록
%   coarse(재포착): 광량 손실(SUM < 93%) 0.2 s 이상, 또는 추정값 NaN 0.5 s 이상
%     원인: 전환 순간 SUM < 93%면 광량 손실, 아니면 NaN. coarse 이벤트는 평균에서 제외
%   결과표: 이벤트를 NaN 없음 / NaN 있음 / 전체로 나눠 평균
%   같은 seed = 같은 외란 순서, 같은 진동
% 전제: Kinesis 앱 꺼짐, laser_on으로 mMTIDevice 연결된 상태 (없으면 직접 연결)

%% 0. 설정
cfg.version  = 7;              % 6 = 큰 빔 재측정 (11/20 렌즈 재삽입, 3.1 mm)   % [수정]
cfg.est_list = [1 2 5 6 7];    % [수정] seed 1
cfg.seed     = 1;              % 외란 순서 seed   % [수정]

cfg.fit_half = 0.3;            % [mm] 정규화 피팅 범위 (큰 빔)   % [수정]
cfg.J        = [40.900 -0.254; 0.416 37.800];   % [mm/u] 11/20 18:24 E3 (간섭 -0.62 %, +1.10 %)   % [수정]

cfg.T_total  = 80.0;           % [s] 추정기당 시간
cfg.t_first  = 4.0;            % [s] 첫 외란 시각 (앞 2 s 정착, 뒤 2 s 정상 상태 흔들림)
cfg.pre_win  = 2.0;            % [s] 정상 상태 흔들림 구간: 첫 외란 전 이만큼
cfg.dt_ev    = [3.0 4.0];      % [s] 외란 간격 범위 (정착 후 진동만 잡는 구간 확보)
cfg.r_mm     = [0.2 3.7];      % [mm] 한 번에 미는 거리 범위 (u_lim 0.1에서 Y 최대 3.78)
cfg.Kp       = 0.4;            % 기준 PID (루프 이득, tune_fine + TEST_fine으로 선정)
cfg.Ki       = 20;             % [1/s] 기준 PID (루프 이득)
cfg.vib_um   = 15;             % [µm] SILEX 진동 RMS (QPD 위, 축당)
cfg.vib_fc   = 1;              % [Hz] SILEX 꺾임 주파수
cfg.Ts_vib   = 0.0158;         % [s] 진동 신호 갱신 간격 (KPA 주기)
cfg.u_lim    = 0.1;            % [u] 명령 한계 (QPD 위 X ±4.09, Y ±3.78 mm)
cfg.loss_ratio = 0.93;         % 광량 손실 기준 (시작 SUM 대비)
cfg.loss_hold  = 0.2;          % [s] 광량 손실이 이만큼 이어지면 coarse
cfg.nan_hold   = 0.5;          % [s] 추정값 NaN이 이만큼 이어지면 coarse
cfg.sat_delta  = 0.95;         % 추정 포화 기준 |δ_DoS|
cfg.SUM_min  = 0.02;           % [V] 이 아래면 그 추정기 중단

cfg.poll_ms  = 5;              % KPA 폴링 [ms]. 1~5 ms 모두 새 값은 16 ms마다 (11/27 진단), 5 ms로 Kinesis 스레드 부담 감소   % [수정 11/27]
cfg.band_rel = 0.02;           % 정착 밴드: 외란 대비
cfg.band_sig = 3;              % 정착 밴드: 정상 상태 흔들림 rms 배수 (둘 중 큰 값)
cfg.n_out    = 3;              % 연속 몇 샘플 밖이면 '안 정착'으로 볼지 (튐 무시)
cfg.Nmax     = 300000;
cfg.floor_k  = 2;              % 추적 구간: 반대쪽 반 신호 >= floor_k × 바닥값

L_mm = 202.5;                  % [mm] 미러-QPD 거리 (µrad 환산)
est_names  = {'DoS', 'LogRatio', 'Hybrid a=0.9', 'Hybrid a=0.8', 'Hybrid a=0.5', 'Hybrid a=0.3', 'Hybrid a=0.1'};   % [수정] 5~7 추가
alpha_list = [1 0 0.9 0.8 0.5 0.3 0.1];                                                                          % [수정]
kin = 'C:\Program Files\Thorlabs\Kinesis';
clear RES

%% 1. 경로
here = fileparts(mfilename('fullpath'));  if isempty(here), here = pwd; end
root = here;
while ~isfolder(fullfile(root, 'Main'))
    parent = fileparts(root);
    if strcmp(parent, root), error('프로젝트 루트(Main 폴더)를 찾지 못함'); end
    root = parent;
end
addpath(genpath(root));
save_dir = fullfile(root, 'Main', 'DATA');

%% 2. 보정된 빔 위치 환산표(추적 구간), 광량 손실 위치 (최신 빔 스캔)   % [수정 11/27]
[TX, lossX] = true_table(save_dir, 'X', cfg.loss_ratio, cfg.floor_k);
[TY, lossY] = true_table(save_dir, 'Y', cfg.loss_ratio, cfg.floor_k);
fprintf('추적 구간(보정된 빔 위치 측정 가능): X %.2f ~ %+.2f mm, Y %.2f ~ %+.2f mm\n', ...   % [수정 11/27]
    TX.P(1), TX.P(end), TY.P(1), TY.P(end));
fprintf('바닥값(반대쪽 반 신호): X %.4f / %.4f V, Y %.4f / %.4f V\n', TX.floor, TY.floor);
track_mm = min(abs([TX.P(1) TX.P(end) TY.P(1) TY.P(end)]));
fprintf('광량 손실(중심 SUM의 %.0f %%) 위치: X %.2f / %+.2f mm, Y %.2f / %+.2f mm\n', ...
    100*cfg.loss_ratio, lossX, lossY);

%% 3. 랜덤 외란 일정 + SILEX 진동 (추정기 공통)
rs = RandStream('mt19937ar', 'Seed', cfg.seed);
ev_t = [];  ev_d = zeros(2, 0);
tt = cfg.t_first;
while tt < cfg.T_total - 0.8
    r  = cfg.r_mm(1) + diff(cfg.r_mm)*rand(rs);
    th = 2*pi*rand(rs);
    ev_t(end+1) = tt;                          %#ok<SAGROW>
    ev_d(:, end+1) = r*[cos(th); sin(th)];     %#ok<SAGROW>
    tt = tt + cfg.dt_ev(1) + diff(cfg.dt_ev)*rand(rs);
end
ev_cum_u = cfg.J \ cumsum(ev_d, 2);
rv = RandStream('mt19937ar', 'Seed', 1000 + cfg.seed);
av = exp(-2*pi*cfg.vib_fc*cfg.Ts_vib);  Nv = ceil(cfg.T_total/cfg.Ts_vib) + 1;  Nb = round(10/cfg.Ts_vib);
vib_mm = zeros(Nv, 2);
for k = 1:2
    v = filter(sqrt(1 - av^2), [1 -av], randn(rv, Nv + Nb, 1));
    vib_mm(:,k) = 1e-3*cfg.vib_um * v(Nb+1:end);
end
vib_u = (cfg.J \ vib_mm.').';
fprintf('seed %d, 외란 %d개 (%.2f~%.2f mm), Kp %.2f, Ki %.0f\n', cfg.seed, numel(ev_t), ...
    min(vecnorm(ev_d)), max(vecnorm(ev_d)), cfg.Kp, cfg.Ki);
fprintf('SILEX 진동: 목표 %.1f µm, 실제 RMS X %.1f / Y %.1f µm\n', cfg.vib_um, 1e3*rms(vib_mm));

%% 4. MZ
if ~exist('mMTIDevice', 'var')
    mMTIDevice = MTIDevice;
    mMTIDevice.ConnectDevice();
    mMTIDevice.ResetDevicePosition();
    mMTIDevice.SetDeviceParam(MTIParam.VdifferenceMax, 173);
    mMTIDevice.SetDeviceParam(MTIParam.HardwareFilterBw, 120);
    mMTIDevice.SetDeviceParam(MTIParam.Vbias, 90);
    mMTIDevice.SetDeviceParam(MTIParam.DataScale, 1.0);
    mMTIDevice.SetDeviceParam(MTIParam.SyncMode, MTISync.Output_DOut0);
    mMTIDevice.SetDeviceParam(MTIParam.DigitalOutputEnable, 1);
    mMTIDevice.SetDeviceParam(MTIParam.MEMSDriverEnable, 1);
end

%% 5. KPA101 (Monitor)
RES = struct('est_id', {}, 'L', {}, 'status', {}, 'SUM0', {}, 'noise_um', {}, 'EV', {}, 'EVe', {}, 'NV', {}, 'tot', {});
stamp = datestr(now, 'yyyymmdd_HHMMSS');   % [수정] 임시 파일 이름용 (이 실행 공통)
try
    kpa = kpa_open(kin, cfg.poll_ms);   % [수정] 연결·초기화를 함수로 묶음 (추정기마다 다시 연결)
    mMTIDevice.GoToDevicePosition(0, 0, 0, 20); pause(1);
    off = mean(read_n(kpa, 20, 5), 1);
    fprintf('dark: XDiff %.4f, YDiff %.4f, SUM %.4f V\n', off);

    mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);
    mMTIDevice.SendDataStream(zeros(1,100), zeros(1,100), 255*ones(1,100), 100, 0, 0);
    mMTIDevice.StartDataStream(-1);
    pause(1);

    %% 6. 추정기별 폐루프
    for ie = 1:numel(cfg.est_list)
        id = cfg.est_list(ie);
        
        a  = alpha_list(id);
        nX = est_norm(save_dir, 'X', a, cfg.fit_half);
        nY = est_norm(save_dir, 'Y', a, cfg.fit_half);
        mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);  pause(1);
        D0 = read_n(kpa, 20, 5) - off;  SUM0 = mean(D0(:,3));
        fprintf('\n[%d/%d] %s 시작 (시작 SUM %.3f V)\n', ie, numel(cfg.est_list), est_names{id}, SUM0);
        [L, status] = run_loop(mMTIDevice, kpa, cfg, a, [nX; nY], off, SUM0, ev_t, ev_cum_u, vib_u, TX, TY);
        fprintf('    결과: %s, 루프 %.2f ms, KPA 새 값 %.2f ms, coarse %d회 (광량 손실 %d, NaN %d), NaN 샘플 %d개\n', ...
            status, 1e3*median(diff(L.t)), 1e3*median(diff(L.t(L.new))), ...
            sum(L.coarse > 0), sum(L.coarse == 1), sum(L.coarse == 2), sum(L.nan & L.new));
        tot = total_metric(L, ev_t);
        fprintf('    전체 RMS (참고, 계단 복귀 포함): %.1f µm (%.0f µrad), 탐색 구간 %.1f %% 시간, 최대 %.2f mm\n', ...   % [수정 11/27]
            tot.rms_um, tot.rms_um/L_mm*1e3, tot.search_pct, tot.max_mm);
        [EV,  nr, NV] = event_metrics(L, ev_t, ev_d, cfg, 'true');
        [EVe, ~,  ~ ] = event_metrics(L, ev_t, ev_d, cfg, 'est');
        RES(ie) = struct('est_id', id, 'L', L, 'status', status, 'SUM0', SUM0, ...
                         'noise_um', 1e3*nr, 'EV', EV, 'EVe', EVe, 'NV', NV, 'tot', tot);
        R = RES(ie);   % [수정] 추정기 하나 끝날 때마다 바로 저장 (꺼져도 끝난 것은 남음)
        part = fullfile(save_dir, sprintf('E5_PART_seed%d_est%d_%s.mat', cfg.seed, id, stamp));   % [수정]
        save(part, 'R', 'cfg', 'alpha_list', 'off', 'ev_t', 'ev_d', 'est_names', ...   % [수정]
            'TX', 'TY', 'lossX', 'lossY', 'track_mm', 'vib_mm');   % [수정]
        fprintf('    임시 저장: %s\n', part);   % [수정]
    end
catch ME
    fprintf('\n에러: %s\n', ME.message);
end

%% 7. 정리 (항상 실행)
try mMTIDevice.StopDataStream(); catch, end
try mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0); catch, end
try mMTIDevice.GoToDevicePosition(0, 0, 0, 20); catch, end
if exist('kpa', 'var'), kpa_close(kpa); end   % [수정]
clear kpa

%% 8. 비교표, NaN 상황표, 그림 (표와 그림은 e5_table, e5_plot: E5_summary와 공용)
if ~isempty(RES)
    for ie = 1:numel(RES), RES(ie).EV.seed = repmat(cfg.seed, height(RES(ie).EV), 1); end
    for ie = 1:numel(RES), RES(ie).NV.seed = repmat(cfg.seed, height(RES(ie).NV), 1); end
    S  = e5_table(RES, 'EV',  est_names);
    Se = e5_table(RES, 'EVe', est_names);
    % 지표 설명 출력 삭제   % [수정 11/27]
    fprintf('\n=== [보정된 빔 위치 기준] seed %d, Kp %.2f, Ki %.0f, 진동 %.0f µm ===\n', cfg.seed, cfg.Kp, cfg.Ki, cfg.vib_um);   % [수정 11/27]
    disp(S)
    % 정규화 X̂ 기준 표는 계산·저장만, 출력 삭제   % [수정 11/27]
    NVall = table();
    for ie = 1:numel(RES)
        if height(RES(ie).NV) == 0, continue; end
        T = RES(ie).NV;  T.estimator = repmat(string(est_names{RES(ie).est_id}), height(T), 1);
        NVall = [NVall; movevars(T, 'estimator', 'Before', 1)];                                     %#ok<AGROW>
    end
    % NaN 상황표는 저장만, 출력 삭제   % [수정 11/27]
    % 추정기별 상태 재출력 삭제 (루프 중 출력과 중복)   % [수정 11/27]
    fprintf('\n=== 정상 상태 (정착 후, 보정된 빔 위치, coarse 제외, 이벤트 제곱평균의 제곱근) ===\n');   % [수정 11/27]
    fprintf('RMSE² = 편향² + 흔들림² = RMSE_x² + RMSE_y² (이 표에서 성립)\n');   % [수정 11/27]
    fprintf('%-14s %9s %9s %9s %9s %9s %6s\n', '', 'RMSE', '편향', '흔들림', 'RMSE_x', 'RMSE_y', 'n');   % [수정 11/27]
    for ie = 1:numel(RES)   % [수정 11/27]
        E = RES(ie).EV;  ok = E.coarse == 0 & isfinite(E.rmse_um);   % [수정 11/27]
        v = sqrt(mean([E.rmse_um(ok), E.bias_um(ok), E.jit_um(ok), E.rmse_x_um(ok), E.rmse_y_um(ok)].^2, 1));   % 이벤트 제곱평균의 제곱근   % [수정 11/27]
        fprintf('%-14s %6.2f um %6.2f um %6.2f um %6.2f um %6.2f um %6d\n', est_names{RES(ie).est_id}, v, sum(ok));   % [수정 11/27]
        fprintf('%-14s %5.1f µrad %5.1f µrad %5.1f µrad %5.1f µrad %5.1f µrad\n', '', v/L_mm*1e3);   % [수정 11/27]
    end   % [수정 11/27]

    out = fullfile(save_dir, sprintf('E5_CMP_seed%d_%s.mat', cfg.seed, datestr(now, 'yyyymmdd_HHMMSS')));
    save(out, 'RES', 'S', 'Se', 'NVall', 'cfg', 'off', 'ev_t', 'ev_d', 'est_names', ...
        'TX', 'TY', 'lossX', 'lossY', 'track_mm', 'vib_mm');
    fprintf('저장: %s\n', out);

    loss_mm = min(abs([lossX lossY]));
    e5_plot(RES, est_names, track_mm, loss_mm, sprintf('seed %d', cfg.seed));
    e5_rms_plot(RES, est_names, ev_t, ev_d, sprintf('seed %d', cfg.seed));
end

%% local functions
function [L, status] = run_loop(dev, kpa, cfg, a, nrm, off, SUM0, ev_t, ev_cum_u, vib_u, TX, TY)
    N = cfg.Nmax;
    L.t = NaN(N,1); L.raw = NaN(N,3); L.est = NaN(N,2); L.estc = NaN(N,2); L.true = NaN(N,2);
    L.u = NaN(N,2); L.dDoS = NaN(N,2); L.new = false(N,1); L.nan = false(N,1);
    L.loss = false(N,1); L.coarse = zeros(N,1); L.reacq = false(N,1);
    integ = [0; 0];  u_pid = [0; 0];   % [수정 11/27] u_cmd 초기화 삭제 (매 루프 먼저 계산됨)
    u_sent = [NaN; NaN];
    r_prev = [NaN NaN NaN];  t_upd = NaN;  est = [0; 0];  est_c = [0; 0];  tru = [0; 0];  dDoS = [0; 0];
    isn = false;  lossS = false;  t_loss0 = NaN;  t_nan0 = NaN;  reacq = false;
    status = '정상 종료';
    k = 0;  t0 = tic;
while true
        t = toc(t0);
if t >= cfg.T_total || k >= N, break; end
        k = k + 1;
        pause(0.001);   % [수정] KPA 과다 읽기 방지
        r_raw = read_n(kpa, 1, 0);  r = r_raw - off;
        is_new = any(r_raw ~= r_prev);  r_prev = r_raw;
        j = find(ev_t <= t, 1, 'last');
if isempty(j), d_step = [0; 0]; else, d_step = ev_cum_u(:, j); end
        iv = min(floor(t/cfg.Ts_vib) + 1, size(vib_u, 1));
        d_now = d_step + vib_u(iv, :).';
        cz = 0;
if is_new
            sig.XDiff = r(1); sig.YDiff = r(2); sig.SUM = r(3);
            [~, ~, d1x, d1y] = estimator_dos(sig, 1);
            [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
if a == 1                          % DoS만 (0 × NaN = NaN 방지)
                dX = d1x;  dY = d1y;
elseif a == 0                      % LogRatio만
                dX = d2x;  dY = d2y;
else                               % Hybrid (LogRatio가 NaN이면 같이 NaN)
                dX = a*d1x + (1-a)*d2x;  dY = a*d1y + (1-a)*d2y;
end
            est  = [(dX - nrm(1,2))/nrm(1,1); (dY - nrm(2,2))/nrm(2,1)];
            tru  = [interp1(TX.d, TX.P, d2x, 'linear', NaN); interp1(TY.d, TY.P, d2y, 'linear', NaN)];
            dDoS = [d1x; d1y];
if r(3) < cfg.SUM_min
                status = sprintf('중단: SUM %.3f V', r(3));
                L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.true(k,:) = tru; L.new(k) = true; break;
end
            isn   = any(~isfinite(est));
            lossS = r(3) < cfg.loss_ratio*SUM0;
if isn,   if isnan(t_nan0),  t_nan0  = t; end, else, t_nan0  = NaN; end
if lossS, if isnan(t_loss0), t_loss0 = t; end, else, t_loss0 = NaN; end
if ~isn, est_c = est; end          % NaN이면 직전 유효값 유지 (PID 입력)
if reacq && ~isn && ~lossS, reacq = false; end
if ~reacq && ((~isnan(t_loss0) && t - t_loss0 >= cfg.loss_hold) || ...
                          (~isnan(t_nan0)  && t - t_nan0  >= cfg.nan_hold))
if lossS, cz = 1; else, cz = 2; end
                integ = -d_step;  u_pid = integ;  % coarse 대신: 계단 외란만 되돌려 미러 중심으로
                est_c = [0; 0];  reacq = true;  t_loss0 = NaN;  t_nan0 = NaN;
else
                dt_upd = t - t_upd;  if isnan(dt_upd), dt_upd = 0; end
                e_u   = cfg.J \ (-est_c);
                integ = integ + cfg.Ki * dt_upd * e_u;
                u_pid = integ + cfg.Kp * e_u;
end
            t_upd = t;
end
                u_cmd = max(min(u_pid + d_now, cfg.u_lim), -cfg.u_lim);
if any(u_cmd ~= u_sent), dev.SetDeviceParam(MTIParam.OutputOffsets, u_cmd(1), u_cmd(2)); u_sent = u_cmd; end
        L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.estc(k,:) = est_c; L.true(k,:) = tru;
        L.u(k,:) = u_cmd; L.dDoS(k,:) = dDoS; L.new(k) = is_new; L.nan(k) = isn;
        L.loss(k) = lossS; L.coarse(k) = cz; L.reacq(k) = reacq;
end
    n = find(~isnan(L.t), 1, 'last');  if isempty(n), n = 0; end
    fn = fieldnames(L);  for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:n, :); end
end

function [EV, nr, NV] = event_metrics(L, ev_t, ev_d, cfg, which)
% which: 'true' = 보정된 빔 위치 기준, 'est' = 정규화 X̂ 기준   % [수정 11/27]
% EV: 이벤트별 지표, NV: NaN이 난 이벤트의 상황
    tn = L.t(L.new);  rw = L.raw(L.new,:);  sn = rw(:,3);
    if strcmp(which, 'true'), Pt = L.true(L.new,:); else, Pt = L.est(L.new,:); end
    Pe = L.est(L.new,:);  Ptr = L.true(L.new,:);
    dn = L.dDoS(L.new,:);  ln = L.loss(L.new);  nn = L.nan(L.new);  cz = L.coarse(L.new);
    pre = tn >= ev_t(1) - cfg.pre_win & tn < ev_t(1) - 0.1;   % 시작 직후 정착 구간 제외
    nr = sqrt(mean(sum(Pt(pre,:).^2, 2), 'omitnan'));     % 정상 상태 반경 흔들림 rms [mm]
    EV = table();  NV = table();
    if isempty(tn), return; end
    nev = find(ev_t < tn(end), 1, 'last');  if isempty(nev), return; end
    vn = {'event','dist_mm','ang_deg','peak_mm','peak_est_mm','settle_ms','ret_ms','os_pct', ...
          'rmse_um','bias_um','jit_um','rmse_x_um','rmse_y_um', ...   % [수정 11/27]
          'min_SUM','nan_n','nan_ms','coarse','loss','sat','search'};
    EV = table('Size', [nev numel(vn)], 'VariableTypes', ...
        [repmat({'double'}, 1, 17), repmat({'logical'}, 1, 3)], 'VariableNames', vn);   % [수정 11/27]
    nvn = {'event','dist_mm','ang_deg','t_nan_ms','nan_n','nan_ms','x_before_mm','y_before_mm', ...
           'SUM_before','half_zero','coarse_nan'};
    NV = table('Size', [0 numel(nvn)], 'VariableTypes', ...
        [repmat({'double'}, 1, 9), {'string', 'logical'}], 'VariableNames', nvn);
    sum0 = max(mean(sn(pre)), eps);
    dts  = median(diff(tn));
    for jj = 1:nev
        t_end = tn(end) + 1;  if jj < numel(ev_t), t_end = ev_t(jj+1); end
        m  = tn >= ev_t(jj) & tn < t_end;
        tj = tn(m) - ev_t(jj);  P = Pt(m,:);
        d  = norm(ev_d(:,jj));  dh = ev_d(:,jj)/d;  ang = atan2d(ev_d(2,jj), ev_d(1,jj));
        err = sqrt(sum(P.^2, 2));                      % NaN = 탐색 구간 (추적 구간 밖)
        srch = any(isnan(err));
        i_last = find(isnan(err), 1, 'last');
        if srch && i_last < numel(tj), ret = 1e3*tj(i_last+1); else, ret = NaN; end
        c = max([0; cz(m)]);                           % 0 없음, 1 광량 손실 coarse, 2 NaN coarse
        band = max(cfg.band_rel*d, cfg.band_sig*nr);
        out = isnan(err) | err > band;
        st = 0;  ist = 1;  run = 0;  last_end = 0;
        for i = 1:numel(out)
            if out(i), run = run + 1; else, run = 0; end
            if run >= cfg.n_out || (out(i) && all(out(1:i))), last_end = i; end
        end
        if last_end > 0
            i2 = last_end;  while i2 < numel(out) && out(i2+1), i2 = i2 + 1; end
            if i2 < numel(tj), st = tj(i2+1); ist = i2 + 1; else, st = NaN; ist = []; end
        end
        if c > 0, st = NaN; ist = []; ret = NaN; end   % coarse 이벤트는 평균에서 제외
        proj = P * dh;  [pk, ipk] = max(proj);
        if isnan(pk) || c > 0, os = NaN; else, os = max(0, -min(proj(ipk:end))) / d * 100; end
        if isempty(ist), rm = NaN; else, rm = 1e3*sqrt(mean(err(ist:end).^2, 'omitnan')); end
        bi = NaN; ji = NaN; rx = NaN; ry = NaN;   % [수정 11/27]
        if ~isempty(ist)   % [수정 11/27]
            Ps = P(ist:end,:);  Ps = Ps(all(isfinite(Ps), 2), :);   % 정착 후, 측정 가능한 샘플만   % [수정 11/27]
            if ~isempty(Ps)   % [수정 11/27]
                mu = mean(Ps, 1);  sg = std(Ps, 1, 1);           % 평균(편향), 표준편차(N 기준)   % [수정 11/27]
                bi = 1e3*norm(mu);  ji = 1e3*norm(sg);   % [수정 11/27]
                rx = 1e3*sqrt(mean(Ps(:,1).^2));  ry = 1e3*sqrt(mean(Ps(:,2).^2));   % [수정 11/27]
            end   % [수정 11/27]
        end   % [수정 11/27]
        nm = nn(m);  nan_n = sum(nm);
        EV(jj,:) = {jj, d, ang, max(err, [], 'includenan'), max(sqrt(sum(Pe(m,:).^2, 2))), 1e3*st, ret, os, rm, ...
                    bi, ji, rx, ry, ...   % [수정 11/27]
                    min(sn(m))/sum0, nan_n, 1e3*dts*nan_n, c, any(ln(m)), ...
                    any(any(abs(dn(m,:)) >= cfg.sat_delta)), srch};
        if nan_n > 0
            idx = find(m);  i1 = idx(find(nm, 1));
            ib  = idx(find(~nm & idx < i1, 1, 'last'));
            if isempty(ib), xb = NaN; yb = NaN; sb = NaN; else, xb = Ptr(ib,1); yb = Ptr(ib,2); sb = sn(ib)/sum0; end
            S1 = rw(i1,3);  hz = strings(0);
            if (S1 + rw(i1,1))/2 <= 0, hz(end+1) = "좌"; end   % XDiff = 좌 - 우
            if (S1 - rw(i1,1))/2 <= 0, hz(end+1) = "우"; end
            if (S1 + rw(i1,2))/2 <= 0, hz(end+1) = "상"; end   % YDiff = 상 - 하
            if (S1 - rw(i1,2))/2 <= 0, hz(end+1) = "하"; end
            NV = [NV; {jj, d, ang, 1e3*(tn(i1) - ev_t(jj)), nan_n, 1e3*dts*nan_n, xb, yb, sb, ...
                       strjoin(hz, ','), c == 2}];                                                  %#ok<AGROW>
        end
    end
end

function tot = total_metric(L, ev_t)
% 첫 외란부터 끝까지 전체 오차 (보정된 빔 위치, 참고용: 계단 복귀 포함).   % [수정 11/27] % 탐색 구간(NaN)은 RMS에서 빼고 시간 비율로 기록
    w = L.new & L.t >= ev_t(1);
    e = sqrt(sum(L.true(w,:).^2, 2));
    tot.rms_um     = 1e3*sqrt(mean(e.^2, 'omitnan'));
    tot.search_pct = 100*mean(isnan(e));
    tot.max_mm     = max(e);
end

function [T, lossP] = true_table(data_dir, ax, loss_ratio, floor_k)
% 빔 스캔에서 LogRatio δ -> 보정된 빔 위치 환산표 (추적 구간만)   % [수정 11/27]
f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
if isempty(f), error('빔 스캔 파일 없음 (%s축)', ax); end
[~, i] = max([f.datenum]);
B = load(fullfile(f(i).folder, f(i).name));
[P, idx] = sort(B.X(:));
sig.XDiff = B.M(idx,1); sig.YDiff = B.M(idx,2); sig.SUM = B.M(idx,3);
[~, ~, d2x, d2y] = estimator_logratio(sig, 1);
if strcmp(ax, 'X'), d = d2x; D = sig.XDiff; else, d = d2y; D = -sig.YDiff; end
dark_p = (sig.SUM + D)/2;        % 빔이 +쪽일 때 반대쪽 반
dark_m = (sig.SUM - D)/2;        % 빔이 -쪽일 때 반대쪽 반
fl = [min(dark_m(P < 0)), min(dark_p(P > 0))];
[~, i0] = min(abs(P));
ip = i0;  while ip < numel(P) && d(ip+1) > d(ip) && dark_p(ip+1) >= floor_k*fl(2), ip = ip + 1; end
im = i0;  while im > 1 && d(im-1) < d(im) && dark_m(im-1) >= floor_k*fl(1), im = im - 1; end
T.P = P(im:ip);  T.d = d(im:ip);  T.floor = fl;
S = sig.SUM;  S0 = interp1(P, S, 0);
lossP = [NaN NaN];
k = find(P < 0 & S < loss_ratio*S0, 1, 'last');  if ~isempty(k), lossP(1) = P(k); end
k = find(P > 0 & S < loss_ratio*S0, 1, 'first'); if ~isempty(k), lossP(2) = P(k); end
end

function D = read_n(kpa, N, poll_ms)
D = NaN(N, 3);
for i = 1:N
    st = kpa.Status;
    D(i,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
              System.Convert.ToDouble(st.PositionDifference.Y), ...
              System.Convert.ToDouble(st.Sum)];
    if poll_ms > 0, pause(poll_ms/1000); end
end
end

function sc = est_norm(data_dir, ax, a, fit_half)
f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
if isempty(f), error('빔 스캔 파일 없음 (%s축)', ax); end
[~, i] = max([f.datenum]);
B = load(fullfile(f(i).folder, f(i).name));
P = B.X(:);
sig.XDiff = B.M(:,1); sig.YDiff = B.M(:,2); sig.SUM = B.M(:,3);
[~, ~, d1x, d1y] = estimator_dos(sig, 1);
[~, ~, d2x, d2y] = estimator_logratio(sig, 1);
if strcmp(ax, 'X'), d = a*d1x + (1-a)*d2x; else, d = a*d1y + (1-a)*d2y; end
m = abs(P) <= fit_half;
p = polyfit(P(m), d(m), 1);
sc = [p(1) p(2)];
end

function kpa = kpa_open(kin, poll_ms)   % [수정] KPA101 연결 + Monitor 모드
NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);
kpa.WaitForSettingsInitialized(10000);
kpa.StartPolling(poll_ms); pause(0.5);
kpa.EnableDevice(); pause(0.5);
mt = asm.AssemblyHandle.GetType( ...
    'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
kpa.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false); pause(0.5);
end

function kpa_close(kpa)   % [수정] 폴링 스레드가 멈춘 뒤에 끊기
try kpa.StopPolling(); catch, end
pause(0.5);                                   % [수정]
try kpa.Disconnect(true); catch, end
pause(0.5);                                   % [수정]
end