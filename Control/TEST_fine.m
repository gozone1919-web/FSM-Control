%% TEST_fine.m
% Fine tracking 하드웨어 확인: SILEX 연속 외란 아래 PI 후보 비교
%   후보마다: 정착 3 s(버림) → 외란 없이 10 s → SILEX 외란 30 s
%   순서 0 A B C C B A 0 (드리프트 확인), 외란 신호는 모든 후보에 같은 것
%   추정기 DoS (중심 근처라 4종 동일), 지표 = QPD로 잰 위치 (센서 노이즈 포함 → 상대 비교)
% 전제: Kinesis 앱 꺼짐, 레이저 켜고 1~2분 예열

%% 0. 설정
cand = struct('name', {'PID 끔', 'A 선정', 'B 기준', 'C SIMC'}, ...
              'Kp',   {NaN, 0.4, 0.5, 0.19414}, 'Ki', {NaN, 20, 20, 18.315});
order    = [1 2 3 4 4 3 2 1];
T_set    = 3;  T_quiet = 10;  T_dist = 30;
amp_um   = 15;  f_c = 1;  seed = 1;       % SILEX 외란 RMS (축당), 꺾임 주파수 [Hz]
Ts       = 0.0158;  L_mm = 202.5;
loss_ratio = 0.93;  loss_hold = 0.2;
J        = [40.900 -0.296; 0.567 37.800];
u_lim    = 0.1;  fit_half = 0.3;  poll_ms = 1;  Nmax = 40000;
kin = 'C:\Program Files\Thorlabs\Kinesis';

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
nX = est_norm(save_dir, 'X', fit_half);  nY = est_norm(save_dir, 'Y', fit_half);

%% 2. SILEX 외란 (Ts 간격, 모든 후보 공통)
rs = RandStream('mt19937ar', 'Seed', seed);
a  = exp(-2*pi*f_c*Ts);  Nd = ceil(T_dist/Ts) + 1;  Nb = round(10/Ts);
Dmm = zeros(Nd, 2);
for k = 1:2
    d = filter(sqrt(1 - a^2), [1 -a], randn(rs, Nd + Nb, 1));
    Dmm(:,k) = 1e-3*amp_um * d(Nb+1:end);
end
Du = (J \ Dmm.').';                        % [u]
fprintf('SILEX 외란: 목표 %.1f µm, 실제 RMS X %.1f / Y %.1f µm\n', amp_um, 1e3*rms(Dmm));

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

RUN = struct('cand', {}, 'L', {}, 'status', {}, 'SUM0', {});
try
    kpa.WaitForSettingsInitialized(10000);
    kpa.StartPolling(poll_ms); pause(0.5);
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

    %% 5. 후보별 실행
    for r = 1:numel(order)
        c = cand(order(r));
        mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);  pause(1);
        D0 = read_n(kpa, 20, 5) - off;  SUM0 = mean(D0(:,3));
        fprintf('[%d/%d] %s 시작 (SUM %.3f V)\n', r, numel(order), c.name, SUM0);
        [L, status] = run_fine(mMTIDevice, kpa, c, off, SUM0, [nX; nY], Du, Ts, J, u_lim, ...
                               T_set, T_quiet, T_dist, loss_ratio, loss_hold, Nmax);
        fprintf('    %s, KPA 새 값 %.2f ms\n', status, 1e3*median(diff(L.t(L.new))));
        RUN(r) = struct('cand', order(r), 'L', L, 'status', status, 'SUM0', SUM0);
    end
catch ME
    fprintf('\n에러: %s\n', ME.message);
end

%% 6. 정리 (항상 실행)
try mMTIDevice.StopDataStream(); catch, end
try mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0); catch, end
try mMTIDevice.GoToDevicePosition(0, 0, 0, 20); catch, end
try kpa.StopPolling(); kpa.Disconnect(true); catch, end
clear kpa

%% 7. 결과
if ~isempty(RUN)
    T = table();
    for r = 1:numel(RUN)
        L = RUN(r).L;  c = cand(RUN(r).cand);
        q = metric(L, T_set, T_set + T_quiet);
        w = metric(L, T_set + T_quiet, T_set + T_quiet + T_dist);
        T = [T; table(r, string(c.name), c.Kp, c.Ki, q(2), w(1), w(2), w(3), w(1)/L_mm*1e3, ...
             string(RUN(r).status), 'VariableNames', {'run', 'cand', 'Kp', 'Ki', 'quiet_sigma_um', ...
             'dist_RMS_um', 'dist_sigma_um', 'dist_bias_um', 'dist_RMS_urad', 'status'})]; %#ok<AGROW>
    end
    s_off = mean(T.dist_sigma_um(T.cand == "PID 끔"));
    T.ratio = T.dist_sigma_um / s_off;
    fprintf('\n=== Fine tracking 하드웨어 (SILEX %.0f µm, 측정 위치 기준, 반경) ===\n', amp_um);
    fprintf('quiet: 외란 없이 정상 상태 흔들림, ratio: σ / (PID 끔 σ), 시뮬레이션 RMS_ratio 약 0.55\n');
    disp(T)
    G = groupsummary(T(T.cand ~= "PID 끔", :), 'cand', 'mean', {'quiet_sigma_um', 'dist_RMS_um', 'ratio'});
    fprintf('=== 후보별 평균 (2회) ===\n');  disp(G)

    out = fullfile(save_dir, sprintf('FINE_HW_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
    save(out, 'RUN', 'T', 'G', 'cand', 'order', 'Dmm', 'off', 'amp_um', 'seed', 'L_mm');
    fprintf('저장: %s\n', out);

    figure('Name', 'Fine HW', 'Color', 'w');
    b = bar(categorical(T.run), [T.dist_RMS_um T.quiet_sigma_um]);  grid on;
    set(gca, 'XTickLabel', compose('%d %s', T.run, T.cand));
    ylabel('µm'); legend(b, {'SILEX 외란 RMS', '외란 없음 σ'}, 'Location', 'best');
    title(sprintf('후보별 흔들림 (SILEX %.0f µm, 측정 위치)', amp_um));
end

%% local functions
function [L, status] = run_fine(dev, kpa, c, off, SUM0, nrm, Du, Ts, J, u_lim, ...
                                T_set, T_quiet, T_dist, loss_ratio, loss_hold, N)
    L.t = NaN(N,1); L.raw = NaN(N,3); L.est = NaN(N,2); L.u = NaN(N,2); L.new = false(N,1);
    usePID = ~isnan(c.Kp);
    integ = [0; 0];  u_pid = [0; 0];  r_prev = [NaN NaN NaN];  t_upd = NaN;  est = [0; 0];
    t_loss0 = NaN;  status = '정상 종료';  T_end = T_set + T_quiet + T_dist;
    k = 0;  t0 = tic;
    while true
        t = toc(t0);
        if t >= T_end || k >= N, break; end
        k = k + 1;
        r_raw = read_n(kpa, 1, 0);  r = r_raw - off;
        is_new = any(r_raw ~= r_prev);  r_prev = r_raw;
        td = t - T_set - T_quiet;
        if td < 0, d_now = [0; 0]; else, d_now = Du(min(floor(td/Ts) + 1, size(Du,1)), :).'; end
        if is_new
            sig.XDiff = r(1); sig.YDiff = r(2); sig.SUM = r(3);
            [~, ~, dx, dy] = estimator_dos(sig, 1);
            est = [(dx - nrm(1,2))/nrm(1,1); (dy - nrm(2,2))/nrm(2,1)];
            if r(3) < loss_ratio*SUM0
                if isnan(t_loss0), t_loss0 = t; end
                if t - t_loss0 >= loss_hold
                    status = sprintf('중단: 광량 손실 (SUM %.3f V)', r(3));
                    L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.new(k) = true; break;
                end
            else
                t_loss0 = NaN;
            end
            if usePID
                dt_upd = t - t_upd;  if isnan(dt_upd), dt_upd = 0; end
                e_u   = J \ (-est);
                integ = integ + c.Ki * dt_upd * e_u;
                u_pid = integ + c.Kp * e_u;
            end
            t_upd = t;
        end
        u_cmd = max(min(u_pid + d_now, u_lim), -u_lim);
        dev.SetDeviceParam(MTIParam.OutputOffsets, u_cmd(1), u_cmd(2));
        L.t(k) = t; L.raw(k,:) = r; L.est(k,:) = est; L.u(k,:) = u_cmd; L.new(k) = is_new;
    end
    n = find(~isnan(L.t), 1, 'last');  if isempty(n), n = 0; end
    fn = fieldnames(L);  for i = 1:numel(fn), L.(fn{i}) = L.(fn{i})(1:n, :); end
end

function m = metric(L, t1, t2)
% [반경 RMS, 반경 σ, |bias|] [µm], 새 샘플만, 구간 [t1, t2)
    w = L.new & L.t >= t1 & L.t < t2;
    if nnz(w) < 10, m = [NaN NaN NaN]; return; end
    P = L.est(w,:);  b = mean(P, 1);
    m = 1e3*[sqrt(mean(sum(P.^2, 2))), sqrt(mean(sum((P - b).^2, 2))), norm(b)];
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

function sc = est_norm(data_dir, ax, fit_half)
% DoS 중심 기울기 정규화 (최신 빔 스캔)
f = dir(fullfile(data_dir, sprintf('BEAM_SCAN_%s_*.mat', ax)));
if isempty(f), error('빔 스캔 파일 없음 (%s축)', ax); end
[~, i] = max([f.datenum]);
B = load(fullfile(f(i).folder, f(i).name));
P = B.X(:);
sig.XDiff = B.M(:,1); sig.YDiff = B.M(:,2); sig.SUM = B.M(:,3);
[~, ~, dx, dy] = estimator_dos(sig, 1);
if strcmp(ax, 'X'), d = dx; else, d = dy; end
m = abs(P) <= fit_half;
p = polyfit(P(m), d(m), 1);
sc = [p(1) p(2)];
end