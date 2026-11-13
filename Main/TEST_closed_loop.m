%% TEST_closed_loop.m
% 실제 빔 폐루프 시험: 랜덤 숨은 스텝 외란 반복, 추정기 여러 개 연속 비교
%   제어: 각 추정기의 X̂ / 평가: 실제 위치(LogRatio δ -> 빔 스캔 표 역변환)와 정규화 X̂ 두 기준
%   추적 구간: 반대쪽 반 신호가 바닥값의 2배 이상인 범위 (실제 위치 측정 가능)
%   탐색 구간: 그 밖 (방향만 앎, 실제 위치 NaN)
%   추정값 NaN: PID는 직전 유효값으로 계속 계산, NaN 샘플·상황은 따로 기록               % [수정]
%   coarse(재포착): 광량 손실(SUM < 93%) 0.2 s 이상, 또는 추정값 NaN 0.5 s 이상          % [수정]
%     원인: 전환 순간 SUM < 93%면 광량 손실, 아니면 NaN. coarse 이벤트는 평균에서 제외    % [수정]
%   결과표: 이벤트를 NaN 없음 / NaN 있음 / 전체로 나눠 평균                                % [수정]
%   같은 seed = 같은 외란 순서
% 전제: Kinesis 앱 꺼짐, laser_on으로 mMTIDevice 연결된 상태 (없으면 직접 연결)

%% 0. 설정
cfg.version  = 2;              % 판정 기준 버전 (E5_summary는 같은 버전끼리만 합침)        % [수정]
   cfg.est_list = [4 3 2 1];
   cfg.seed     = 3;              % 외란 순서 seed                                            % [수정]
cfg.T_total  = 40.0;           % [s] 추정기당 시간
cfg.t_first  = 2.0;            % [s] 첫 외란 시각
cfg.dt_ev    = [1.0 1.5];      % [s] 외란 간격 범위
cfg.r_mm     = [0.2 3.7];      % [mm] 한 번에 미는 거리 범위 (u_lim 0.1에서 Y 최대 3.78)
cfg.Ki       = 20;             % [1/s] 기준 PID (루프 이득)
cfg.Kp       = 0.5;            % 기준 PID (루프 이득)
cfg.u_lim    = 0.1;            % [u] 명령 한계 (QPD 위 X ±4.09, Y ±3.78 mm)
cfg.loss_ratio = 0.93;         % 광량 손실 기준 (시작 SUM 대비)                             % [수정]
cfg.loss_hold  = 0.2;          % [s] 광량 손실이 이만큼 이어지면 coarse                     % [수정]
cfg.nan_hold   = 0.5;          % [s] 추정값 NaN이 이만큼 이어지면 coarse                     % [수정]
cfg.sat_delta  = 0.95;         % 추정 포화 기준 |δ_DoS|
cfg.SUM_min  = 0.02;           % [V] 이 아래면 그 추정기 중단
cfg.fit_half = 0.3;            % [mm] 정규화 피팅 범위 (init_pid_sim과 같음)
cfg.J        = [40.900 -0.296; 0.567 37.800];   % [mm/u] E3
cfg.poll_ms  = 1;              % KPA 폴링 [ms]
cfg.band_rel = 0.02;           % 정착 밴드: 외란 대비
cfg.band_sig = 3;              % 정착 밴드: 잡음 rms 배수 (둘 중 큰 값)
cfg.n_out    = 3;              % 연속 몇 샘플 밖이면 '안 정착'으로 볼지 (튐 무시)
cfg.Nmax     = 40000;
cfg.floor_k  = 2;              % 추적 구간: 반대쪽 반 신호 >= floor_k × 바닥값
est_names  = {'DoS', 'LogRatio', 'Hybrid a=0.9', 'Hybrid a=0.8'};
alpha_list = [1 0 0.9 0.8];
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

%% 2. 실제 위치 환산표(추적 구간), 광량 손실 위치 (최신 빔 스캔)
[TX, lossX] = true_table(save_dir, 'X', cfg.loss_ratio, cfg.floor_k);
[TY, lossY] = true_table(save_dir, 'Y', cfg.loss_ratio, cfg.floor_k);
fprintf('추적 구간(실제 위치 측정 가능): X %.2f ~ %+.2f mm, Y %.2f ~ %+.2f mm\n', ...
    TX.P(1), TX.P(end), TY.P(1), TY.P(end));
fprintf('바닥값(반대쪽 반 신호): X %.4f / %.4f V, Y %.4f / %.4f V\n', TX.floor, TY.floor);
track_mm = min(abs([TX.P(1) TX.P(end) TY.P(1) TY.P(end)]));                                 % 그림용
fprintf('광량 손실(중심 SUM의 %.0f %%) 위치: X %.2f / %+.2f mm, Y %.2f / %+.2f mm\n', ...
    100*cfg.loss_ratio, lossX, lossY);

%% 3. 랜덤 외란 일정 (추정기 공통)
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
fprintf('seed %d, 외란 %d개 (%.2f~%.2f mm), Kp %.2f, Ki %.0f\n', cfg.seed, numel(ev_t), ...
    min(vecnorm(ev_d)), max(vecnorm(ev_d)), cfg.Kp, cfg.Ki);

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
NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);

RES = struct('est_id', {}, 'L', {}, 'status', {}, 'SUM0', {}, 'noise_um', {}, 'EV', {}, 'EVe', {}, 'NV', {});  % [수정]
try
    kpa.WaitForSettingsInitialized(10000);
    kpa.StartPolling(cfg.poll_ms); pause(0.5);
    kpa.EnableDevice(); pause(0.5);
    mt = asm.AssemblyHandle.GetType( ...
        'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
    kpa.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false); pause(0.5);

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
        [L, status] = run_loop(mMTIDevice, kpa, cfg, a, [nX; nY], off, SUM0, ev_t, ev_cum_u, TX, TY);
        fprintf('    결과: %s, 루프 %.2f ms, KPA 새 값 %.2f ms, coarse %d회 (광량 손실 %d, NaN %d), NaN 샘플 %d개\n', ...  % [수정]
            status, 1e3*median(diff(L.t)), 1e3*median(diff(L.t(L.new))), ...                                     % [수정]
            sum(L.coarse > 0), sum(L.coarse == 1), sum(L.coarse == 2), sum(L.nan & L.new));                      % [수정]
        [EV,  nr, NV] = event_metrics(L, ev_t, ev_d, cfg, 'true');                                               % [수정]
        [EVe, ~,  ~ ] = event_metrics(L, ev_t, ev_d, cfg, 'est');                                                % [수정]
        RES(ie) = struct('est_id', id, 'L', L, 'status', status, 'SUM0', SUM0, ...
                         'noise_um', 1e3*nr, 'EV', EV, 'EVe', EVe, 'NV', NV);                                    % [수정]
    end
catch ME
    fprintf('\n에러: %s\n', ME.message);
end

%% 7. 정리 (항상 실행)
try mMTIDevice.StopDataStream(); catch, end
try mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0); catch, end
try mMTIDevice.GoToDevicePosition(0, 0, 0, 20); catch, end
try kpa.StopPolling(); kpa.Disconnect(true); catch, end
clear kpa

%% 8. 비교표, NaN 상황표, 그림 (표와 그림은 e5_table, e5_plot: E5_summary와 공용)          % [수정]
if ~isempty(RES)
    for ie = 1:numel(RES), RES(ie).EV.seed = repmat(cfg.seed, height(RES(ie).EV), 1); end           % [수정]
    for ie = 1:numel(RES), RES(ie).NV.seed = repmat(cfg.seed, height(RES(ie).NV), 1); end           % [수정]
    S  = e5_table(RES, 'EV',  est_names);                                                           % [수정]
    Se = e5_table(RES, 'EVe', est_names);                                                           % [수정]
    fprintf('\n그룹: NaN없음 / NaN있음 / 전체. 평균·중앙값은 coarse 이벤트를 뺀 값, 횟수 열은 전체\n');  % [수정]
    fprintf('coarse_loss: 광량 손실로 coarse, coarse_nan: NaN으로 coarse, loss: 광량 손실 발생(회복 포함)\n');  % [수정]
    fprintf('search: 탐색 구간 진입, ret_med_ms: 추적 구간 복귀 시간, nan_n: NaN 샘플 수 합\n');      % [수정]
    fprintf('peak_ratio: 탐색 구간에 안 들어간 이벤트만 (들어가면 최대 오차 측정 불가)\n');
    fprintf('\n=== [실제 위치 기준] seed %d, Kp %.2f, Ki %.0f ===\n', cfg.seed, cfg.Kp, cfg.Ki);
    disp(S)
    fprintf('\n=== [정규화 X̂ 기준] seed %d, Kp %.2f, Ki %.0f ===\n', cfg.seed, cfg.Kp, cfg.Ki);
    disp(Se)
    NVall = table();                                                                                % [수정]
    for ie = 1:numel(RES)                                                                           % [수정]
        if height(RES(ie).NV) == 0, continue; end                                                   % [수정]
        T = RES(ie).NV;  T.estimator = repmat(string(est_names{RES(ie).est_id}), height(T), 1);     % [수정]
        NVall = [NVall; movevars(T, 'estimator', 'Before', 1)];                                     %#ok<AGROW> % [수정]
    end                                                                                             % [수정]
    fprintf('\n=== NaN 상황 (NaN이 난 이벤트마다 한 줄) ===\n');                                    % [수정]
    if height(NVall) > 0, disp(NVall), else, disp('NaN 없음'); end                                  % [수정]
    for ie = 1:numel(RES)
        fprintf('%-14s 상태: %s, 잡음 %.2f um rms (실제 위치)\n', est_names{RES(ie).est_id}, RES(ie).status, RES(ie).noise_um);
    end

    out = fullfile(save_dir, sprintf('E5_CMP_seed%d_%s.mat', cfg.seed, datestr(now, 'yyyymmdd_HHMMSS')));
    save(out, 'RES', 'S', 'Se', 'NVall', 'cfg', 'off', 'ev_t', 'ev_d', 'est_names', ...             % [수정]
        'TX', 'TY', 'lossX', 'lossY', 'track_mm');                                                  % [수정]
    fprintf('저장: %s\n', out);

    loss_mm = min(abs([lossX lossY]));                                                              % [수정]
    e5_plot(RES, est_names, track_mm, loss_mm, sprintf('seed %d', cfg.seed));                       % [수정]
end

%% local functions
function [L, status] = run_loop(dev, kpa, cfg, a, nrm, off, SUM0, ev_t, ev_cum_u, TX, TY)
    N = cfg.Nmax;
    L.t = NaN(N,1); L.raw = NaN(N,3); L.est = NaN(N,2); L.estc = NaN(N,2); L.true = NaN(N,2);         % [수정]
    L.u = NaN(N,2); L.dDoS = NaN(N,2); L.new = false(N,1); L.nan = false(N,1);                        % [수정]
    L.loss = false(N,1); L.coarse = zeros(N,1); L.reacq = false(N,1);                                 % [수정]
    integ = [0; 0];  u_pid = [0; 0];  u_cmd = [0; 0];
    r_prev = [NaN NaN NaN];  t_upd = NaN;  est = [0; 0];  est_c = [0; 0];  tru = [0; 0];  dDoS = [0; 0];  % [수정]
    isn = false;  lossS = false;  t_loss0 = NaN;  t_nan0 = NaN;  reacq = false;                       % [수정]
    status = '정상 종료';
    k = 0;  t0 = tic;
    while true
        t = toc(t0);
        if t >= cfg.T_total || k >= N, break; end
        k = k + 1;
        r_raw = read_n(kpa, 1, 0);  r = r_raw - off;
        is_new = any(r_raw ~= r_prev);  r_prev = r_raw;
        j = find(ev_t <= t, 1, 'last');
        if isempty(j), d_now = [0; 0]; else, d_now = ev_cum_u(:, j); end
        cz = 0;                                                                                       % [수정]
        if is_new
            sig.XDiff = r(1); sig.YDiff = r(2); sig.SUM = r(3);
            [~, ~, d1x, d1y] = estimator_dos(sig, 1);
            [~, ~, d2x, d2y] = estimator_logratio(sig, 1);
            if a == 1                          % DoS만 (0 × NaN = NaN 방지)                           % [수정]
                dX = d1x;  dY = d1y;                                                                  % [수정]
            elseif a == 0                      % LogRatio만                                           % [수정]
                dX = d2x;  dY = d2y;                                                                  % [수정]
            else                               % Hybrid (LogRatio가 NaN이면 같이 NaN)                 % [수정]
                dX = a*d1x + (1-a)*d2x;  dY = a*d1y + (1-a)*d2y;                                      % [수정]
            end                                                                                       % [수정]
            est  = [(dX - nrm(1,2))/nrm(1,1); (dY - nrm(2,2))/nrm(2,1)];
            tru  = [interp1(TX.d, TX.P, d2x, 'linear', NaN); interp1(TY.d, TY.P, d2y, 'linear', NaN)];
            dDoS = [d1x; d1y];
            if r(3) < cfg.SUM_min
                status = sprintf('중단: SUM %.3f V', r(3));
                L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.true(k,:) = tru; L.new(k) = true; break;
            end
            % 판정: 광량 손실(SUM)과 NaN을 따로                                                     % [수정]
            isn   = any(~isfinite(est));                                                              % [수정]
            lossS = r(3) < cfg.loss_ratio*SUM0;                                                       % [수정]
            if isn,   if isnan(t_nan0),  t_nan0  = t; end, else, t_nan0  = NaN; end                  % [수정]
            if lossS, if isnan(t_loss0), t_loss0 = t; end, else, t_loss0 = NaN; end                  % [수정]
            if ~isn, est_c = est; end          % NaN이면 직전 유효값 유지 (PID 입력)                 % [수정]
            if reacq && ~isn && ~lossS, reacq = false; end   % coarse 후 정상 복귀                    % [수정]
            % coarse 판정: 광량 손실 0.2 s 또는 NaN 0.5 s. 원인은 전환 순간 SUM으로                % [수정]
            if ~reacq && ((~isnan(t_loss0) && t - t_loss0 >= cfg.loss_hold) || ...                    % [수정]
                          (~isnan(t_nan0)  && t - t_nan0  >= cfg.nan_hold))                           % [수정]
                if lossS, cz = 1; else, cz = 2; end                                                   % [수정]
                integ = -d_now;  u_pid = integ;  % coarse 대신: 명령을 미러 중심으로                  % [수정]
                est_c = [0; 0];  reacq = true;  t_loss0 = NaN;  t_nan0 = NaN;                         % [수정]
            else                                                                                      % [수정]
                dt_upd = t - t_upd;  if isnan(dt_upd), dt_upd = 0; end                                % [수정]
                e_u   = cfg.J \ (-est_c);                                                             % [수정]
                integ = integ + cfg.Ki * dt_upd * e_u;                                                % [수정]
                u_pid = integ + cfg.Kp * e_u;                                                         % [수정]
            end                                                                                       % [수정]
            t_upd = t;
        end
        u_cmd = max(min(u_pid + d_now, cfg.u_lim), -cfg.u_lim);
        dev.SetDeviceParam(MTIParam.OutputOffsets, u_cmd(1), u_cmd(2));
        L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.estc(k,:) = est_c; L.true(k,:) = tru;        % [수정]
        L.u(k,:) = u_cmd; L.dDoS(k,:) = dDoS; L.new(k) = is_new; L.nan(k) = isn;                      % [수정]
        L.loss(k) = lossS; L.coarse(k) = cz; L.reacq(k) = reacq;                                      % [수정]
    end
    n = find(~isnan(L.t), 1, 'last');  if isempty(n), n = 0; end
    fn = fieldnames(L);  for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:n, :); end
end

function [EV, nr, NV] = event_metrics(L, ev_t, ev_d, cfg, which)                                    % [수정]
% which: 'true' = 실제 위치 기준, 'est' = 정규화 X̂ 기준
% EV: 이벤트별 지표, NV: NaN이 난 이벤트의 상황                                                    % [수정]
    tn = L.t(L.new);  rw = L.raw(L.new,:);  sn = rw(:,3);                                           % [수정]
    if strcmp(which, 'true'), Pt = L.true(L.new,:); else, Pt = L.est(L.new,:); end
    Pe = L.est(L.new,:);  Ptr = L.true(L.new,:);                                                    % [수정]
    dn = L.dDoS(L.new,:);  ln = L.loss(L.new);  nn = L.nan(L.new);  cz = L.coarse(L.new);           % [수정]
    pre = tn < ev_t(1) - 0.1;
    nr = sqrt(mean(sum(Pt(pre,:).^2, 2), 'omitnan'));     % 반경 잡음 rms [mm]
    EV = table();  NV = table();                                                                    % [수정]
    if isempty(tn), return; end
    nev = find(ev_t < tn(end), 1, 'last');  if isempty(nev), return; end
    vn = {'event','dist_mm','ang_deg','peak_mm','peak_est_mm','settle_ms','ret_ms','os_pct', ...   % [수정]
          'rmse_um','min_SUM','nan_n','nan_ms','coarse','loss','sat','search'};                     % [수정]
    EV = table('Size', [nev numel(vn)], 'VariableTypes', ...
        [repmat({'double'}, 1, 13), repmat({'logical'}, 1, 3)], 'VariableNames', vn);              % [수정]
    nvn = {'event','dist_mm','ang_deg','t_nan_ms','nan_n','nan_ms','x_before_mm','y_before_mm', ... % [수정]
           'SUM_before','half_zero','coarse_nan'};                                                  % [수정]
    NV = table('Size', [0 numel(nvn)], 'VariableTypes', ...                                         % [수정]
        [repmat({'double'}, 1, 9), {'string', 'logical'}], 'VariableNames', nvn);                  % [수정]
    sum0 = max(mean(sn(pre)), eps);                                                                 % [수정]
    dts  = median(diff(tn));                       % 새 샘플 간격 [s]                              % [수정]
    for jj = 1:nev
        t_end = tn(end) + 1;  if jj < numel(ev_t), t_end = ev_t(jj+1); end
        m  = tn >= ev_t(jj) & tn < t_end;
        tj = tn(m) - ev_t(jj);  P = Pt(m,:);
        d  = norm(ev_d(:,jj));  dh = ev_d(:,jj)/d;  ang = atan2d(ev_d(2,jj), ev_d(1,jj));           % [수정]
        err = sqrt(sum(P.^2, 2));                      % NaN = 탐색 구간 (추적 구간 밖)
        srch = any(isnan(err));
        i_last = find(isnan(err), 1, 'last');
        if srch && i_last < numel(tj), ret = 1e3*tj(i_last+1); else, ret = NaN; end            % 추적 구간 복귀 시간
        c = max([0; cz(m)]);                           % 0 없음, 1 광량 손실 coarse, 2 NaN coarse    % [수정]
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
        if c > 0, st = NaN; ist = []; ret = NaN; end   % coarse 이벤트는 평균에서 제외             % [수정]
        proj = P * dh;  [pk, ipk] = max(proj);
        if isnan(pk) || c > 0, os = NaN; else, os = max(0, -min(proj(ipk:end))) / d * 100; end      % [수정]
        if isempty(ist), rm = NaN; else, rm = 1e3*sqrt(mean(err(ist:end).^2, 'omitnan')); end
        nm = nn(m);  nan_n = sum(nm);                                                               % [수정]
        EV(jj,:) = {jj, d, ang, max(err, [], 'includenan'), max(sqrt(sum(Pe(m,:).^2, 2))), 1e3*st, ret, os, rm, ...  % [수정]
                    min(sn(m))/sum0, nan_n, 1e3*dts*nan_n, c, any(ln(m)), ...                     % [수정]
                    any(any(abs(dn(m,:)) >= cfg.sat_delta)), srch};                                 % [수정]
        if nan_n > 0                                                                                % [수정]
            idx = find(m);  i1 = idx(find(nm, 1));     % 첫 NaN 샘플                                % [수정]
            ib  = idx(find(~nm & idx < i1, 1, 'last'));  % 그 직전 유효 샘플                        % [수정]
            if isempty(ib), xb = NaN; yb = NaN; sb = NaN; else, xb = Ptr(ib,1); yb = Ptr(ib,2); sb = sn(ib)/sum0; end  % [수정]
            S1 = rw(i1,3);  hz = strings(0);                                                        % [수정]
            if (S1 + rw(i1,1))/2 <= 0, hz(end+1) = "좌"; end   % XDiff = 좌 - 우                    % [수정]
            if (S1 - rw(i1,1))/2 <= 0, hz(end+1) = "우"; end                                        % [수정]
            if (S1 + rw(i1,2))/2 <= 0, hz(end+1) = "상"; end   % YDiff = 상 - 하                    % [수정]
            if (S1 - rw(i1,2))/2 <= 0, hz(end+1) = "하"; end                                        % [수정]
            NV = [NV; {jj, d, ang, 1e3*(tn(i1) - ev_t(jj)), nan_n, 1e3*dts*nan_n, xb, yb, sb, ...  % [수정]
                       strjoin(hz, ','), c == 2}];                                                  %#ok<AGROW> % [수정]
        end                                                                                         % [수정]
    end
end

function [T, lossP] = true_table(data_dir, ax, loss_ratio, floor_k)
% 빔 스캔에서 LogRatio δ -> 실제 위치 환산표 (추적 구간만)
% 추적 구간: 중심에서 바깥으로 가며, 빔 반대쪽 반 신호가 floor_k × 바닥값 이상이고
%            δ가 단조 증가하는 동안. 바닥값 = 그 쪽 스캔에서 반대쪽 반 신호의 최솟값
% lossP: SUM이 중심 SUM의 loss_ratio 아래로 떨어지는 위치 [음수쪽, 양수쪽]
f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
if isempty(f), error('빔 스캔 파일 없음 (%s축)', ax); end
[~, i] = max([f.datenum]);
B = load(fullfile(f(i).folder, f(i).name));
[P, idx] = sort(B.X(:));
sig.XDiff = B.M(idx,1); sig.YDiff = B.M(idx,2); sig.SUM = B.M(idx,3);
[~, ~, d2x, d2y] = estimator_logratio(sig, 1);
if strcmp(ax, 'X'), d = d2x; D = sig.XDiff; else, d = d2y; D = -sig.YDiff; end
% 반대쪽 반 신호 (D: 빔이 +쪽으로 가면 음수가 되도록 부호 맞춤)
%   X: XDiff = 좌-우,  +X일 때 반대쪽 = 좌 = (SUM+XDiff)/2,  -X일 때 = 우 = (SUM-XDiff)/2
%   Y: YDiff = 상-하,  +Y일 때 반대쪽 = 하 = (SUM-YDiff)/2,  -Y일 때 = 상 = (SUM+YDiff)/2
dark_p = (sig.SUM + D)/2;        % 빔이 +쪽일 때 반대쪽 반
dark_m = (sig.SUM - D)/2;        % 빔이 -쪽일 때 반대쪽 반
fl = [min(dark_m(P < 0)), min(dark_p(P > 0))];   % 바닥값 [음수쪽, 양수쪽]
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
