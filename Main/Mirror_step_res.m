%% E4_step_response.m
% E4: 미러 스텝 응답 실측 (폐루프 없음)
%   OutputOffsets로 스텝 명령 -> KPA101로 빔 위치 기록
%   명령 후 빔이 10/50/90 %에 도달하는 시간 측정 (KPA 갱신까지 포함한 루프가 보는 응답)
% 전제: Kinesis 앱 꺼짐, laser_on으로 mMTIDevice 연결된 상태 (없으면 직접 연결)

clear ST R L T10 T50 T90 OS   % 이전 스크립트가 남긴 같은 이름 변수 제거 (mMTIDevice는 유지)

%% 0. 설정
cfg.step_mm  = 0.5;            % [mm] 스텝 크기 (선형 구간)
cfg.n_rep    = 5;              % 반복 횟수 (한 번 = 0 -> + -> 0 -> - -> 0, 스텝 4개)
cfg.hold_s   = 0.4;            % [s] 각 위치 유지 시간
cfg.jitter_s = 0.016;          % [s] 스텝 전 랜덤 대기 최대값 (KPA 갱신 시점과 엇갈리게)
cfg.poll_ms  = 1;              % KPA 폴링 [ms]
cfg.fit_half = 0.3;            % [mm] 정규화 피팅 범위
cfg.J        = [40.900 -0.296; 0.567 37.800];   % [mm/u] E3
cfg.SUM_min  = 0.02;
cfg.Nmax     = 40000;
kin = 'C:\Program Files\Thorlabs\Kinesis';

%% 1. 경로, 정규화 (DoS)
here = fileparts(mfilename('fullpath'));  if isempty(here), here = pwd; end
root = here;
while ~isfolder(fullfile(root, 'Main'))
    parent = fileparts(root);
    if strcmp(parent, root), error('프로젝트 루트(Main 폴더)를 찾지 못함'); end
    root = parent;
end
addpath(genpath(root));
save_dir = fullfile(root, 'Main', 'DATA');
[nX, ~] = est_norm(save_dir, 'X', 1, cfg.fit_half);
[nY, ~] = est_norm(save_dir, 'Y', 1, cfg.fit_half);

%% 2. 스텝 순서 만들기 [축, 목표 mm]
lv = cfg.step_mm * [1 0 -1 0];
seq = zeros(0, 2);
for ax = 1:2
    for k = 1:cfg.n_rep
        seq = [seq; ax*ones(4,1), lv(:)];    %#ok<AGROW>
    end
end
nst = size(seq, 1);

%% 3. MZ
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

%% 4. KPA101 (Monitor)
NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);

L = struct();  status = '정상 종료';
try
    kpa.WaitForSettingsInitialized(10000);
    kpa.StartPolling(cfg.poll_ms); pause(0.5);
    kpa.EnableDevice(); pause(0.5);
    mt = asm.AssemblyHandle.GetType( ...
        'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
    kpa.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false); pause(0.5);

    % dark -> (0,0) 스트림, 레이저 켬
    mMTIDevice.GoToDevicePosition(0, 0, 0, 20); pause(1);
    off = mean(read_n(kpa, 20, 5), 1);
    mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);
    mMTIDevice.SendDataStream(zeros(1,100), zeros(1,100), 255*ones(1,100), 100, 0, 0);
    mMTIDevice.StartDataStream(-1);
    pause(1);
    SUM0 = mean(read_n(kpa, 20, 5) - off, 1);  SUM0 = SUM0(3);
    if SUM0 < 10*cfg.SUM_min, error('시작 SUM이 너무 낮음 (%.3f V)', SUM0); end
    fprintf('E4 스텝 응답: %d개 스텝, 크기 %.2f mm, 시작 SUM %.3f V\n', nst, cfg.step_mm, SUM0);

    % 기록 (연속), 스텝 시각
    N = cfg.Nmax;
    L.t = NaN(N,1); L.raw = NaN(N,3); L.est = NaN(N,2); L.new = false(N,1);
    ST.t0 = NaN(nst,1); ST.t1 = NaN(nst,1); ST.u = NaN(nst,2); ST.ax = seq(:,1); ST.mm = seq(:,2);
    rs = RandStream('mt19937ar', 'Seed', 4);
    r_prev = [NaN NaN NaN];  k = 0;  t0 = tic;
    u_now = [0; 0];
    for s = 0:nst
        % 다음 스텝 전 유지 시간 (+ 랜덤 대기) 동안 계속 읽기
        t_hold = cfg.hold_s + cfg.jitter_s*rand(rs);
        t_end = toc(t0) + t_hold;
        while toc(t0) < t_end && k < N
            r_raw = read_n(kpa, 1, 0);  t = toc(t0);
            k = k + 1;
            is_new = any(r_raw ~= r_prev);  r_prev = r_raw;
            r = r_raw - off;
            sig.XDiff = r(1); sig.YDiff = r(2); sig.SUM = r(3);
            [~, ~, d1x, d1y] = estimator_dos(sig, 1);
            L.t(k) = t;  L.raw(k,:) = r;  L.new(k) = is_new;
            L.est(k,:) = [(d1x - nX(2))/nX(1), (d1y - nY(2))/nY(1)];
            if r(3) < cfg.SUM_min, error('SUM %.3f V < %.3f V', r(3), cfg.SUM_min); end
        end
        if s == nst, break; end
        % 스텝 명령
        tgt = [0; 0];  tgt(seq(s+1,1)) = seq(s+1,2);
        u_now = cfg.J \ tgt;
        ST.t0(s+1) = toc(t0);
        mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, u_now(1), u_now(2));
        ST.t1(s+1) = toc(t0);
        ST.u(s+1,:) = u_now;
    end
catch ME
    status = ['에러: ' ME.message];
end

%% 5. 정리 (항상 실행)
try mMTIDevice.StopDataStream(); catch, end
try mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0); catch, end
try mMTIDevice.GoToDevicePosition(0, 0, 0, 20); catch, end
try kpa.StopPolling(); kpa.Disconnect(true); catch, end
clear kpa
fprintf('결과: %s\n', status);

%% 6. 분석: 스텝별 정규화 응답
n = find(~isnan(L.t), 1, 'last');
fn = fieldnames(L);  for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:n, :); end
tn = L.t(L.new);  en = L.est(L.new, :);
fprintf('KPA 새 값 간격 중앙값 %.2f ms, 명령 전송 시간 중앙값 %.2f ms\n', ...
    1e3*median(diff(tn)), 1e3*median(ST.t1 - ST.t0, 'omitnan'));

prev_mm = [0; ST.mm(1:end-1)];
prev_ax = [ST.ax(1); ST.ax(1:end-1)];
R = struct('tau', {}, 'y', {});
T10 = NaN(nst,1); T50 = T10; T90 = T10; OS = T10;
for s = 1:nst
    if isnan(ST.t0(s)), continue; end
    ax = ST.ax(s);
    from = 0;  if prev_ax(s) == ax, from = prev_mm(s); end
    to = ST.mm(s);  dlt = to - from;
    if abs(dlt) < 1e-6, continue; end
    t_next = Inf;  if s < nst, t_next = ST.t0(s+1); end
    m_bef = tn > ST.t0(s) - 0.1 & tn <= ST.t0(s);
    m_aft = tn > ST.t0(s) & tn < t_next;
    if ~any(m_bef) || ~any(m_aft), continue; end
    base = mean(en(m_bef, ax));
    tau = tn(m_aft) - ST.t0(s);
    y = (en(m_aft, ax) - base) / dlt;                 % 0 -> 1 로 정규화
    R(end+1) = struct('tau', tau, 'y', y);            %#ok<SAGROW>
    T10(s) = first_cross(tau, y, 0.1);
    T50(s) = first_cross(tau, y, 0.5);
    T90(s) = first_cross(tau, y, 0.9);
    OS(s)  = max(0, max(y) - 1) * 100;
end
for ax = 1:2
    m = ST.ax == ax;
    fprintf('[%s축] 10%% %.1f ms, 50%% %.1f ms, 90%% %.1f ms (중앙값), 넘침 중앙값 %.1f %%\n', ...
        char('W' + ax), 1e3*median(T10(m), 'omitnan'), 1e3*median(T50(m), 'omitnan'), ...
        1e3*median(T90(m), 'omitnan'), median(OS(m), 'omitnan'));
end

out = fullfile(save_dir, sprintf('E4_STEP_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(out, 'L', 'ST', 'cfg', 'off', 'SUM0', 'status', 'T10', 'T50', 'T90', 'OS', 'R');
fprintf('저장: %s\n', out);

%% 7. 그림: 모든 스텝 겹쳐 그리기
figure('Name', 'E4 step response', 'Color', 'w');
hold on; grid on;
for i = 1:numel(R), plot(1e3*R(i).tau, R(i).y, '.', 'MarkerSize', 8); end
yline(0.1, ':'); yline(0.5, ':'); yline(0.9, ':'); yline(1, 'k--');
xlim([0 200]); ylim([-0.2 1.3]);
xlabel('명령 후 시간 [ms]'); ylabel('정규화 응답 (0 -> 1)');
title(sprintf('E4 미러 스텝 응답 (OutputOffsets, %d개 스텝, KPA 기준)', numel(R)));

%% local functions
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

function [sc, fname] = est_norm(data_dir, ax, a, fit_half)
f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
if isempty(f), error('빔 스캔 파일 없음 (%s축)', ax); end
[~, i] = max([f.datenum]);  fname = f(i).name;
B = load(fullfile(f(i).folder, fname));
P = B.X(:);
sig.XDiff = B.M(:,1); sig.YDiff = B.M(:,2); sig.SUM = B.M(:,3);
[~, ~, d1x, d1y] = estimator_dos(sig, 1);
[~, ~, d2x, d2y] = estimator_logratio(sig, 1);
if strcmp(ax, 'X'), d = a*d1x + (1-a)*d2x; else, d = a*d1y + (1-a)*d2y; end
m = abs(P) <= fit_half;
p = polyfit(P(m), d(m), 1);
sc = [p(1) p(2)];
end

function tc = first_cross(tau, y, lvl)
i = find(y >= lvl, 1);
if isempty(i), tc = NaN; else, tc = tau(i); end
end