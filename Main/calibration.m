%% e3_calibration.m  (E3)
% S46642 + USB-SL MZ + PDQ80A + KPA101 정적 보정
%   1. FSM X/Y 명령을 조금씩 바꾸며 QPD 신호 측정 (왕복)
%   2. DoS / LogRatio / Hybrid 무차원 δ 계산
%   3. 실측 2x2 감도 행렬 S = dδ/du
%   4. S3 광학 모델 |J|로 k [mm] 환산 (k > 0, δ 부호가 QPD 좌표와 맞다는 전제)
%   5. 잡음, 히스테리시스, 축 간 간섭
% 실행 전: 레이저용 mMTIDevice 연결 해제, MirrorcleDraw / Kinesis 앱 종료, 광학계 정렬·고정

%% 1. 설정
cfg.project_root = fileparts(fileparts(mfilename('fullpath')));   % Main의 상위 = FSM Control
if isempty(cfg.project_root), cfg.project_root = pwd; end
addpath(genpath(cfg.project_root));

cfg.ini_file = which('mtidevice - S46642.ini');
if isempty(cfg.ini_file)
    cfg.ini_file = fullfile(cfg.project_root, 'FSM', 'MirrorcleSDK', 'mtidevice - S46642.ini');
end
cfg.com_port  = '';                          % '' = 자동 연결, 예: 'COM3'
cfg.kinesis   = 'C:\Program Files\Thorlabs\Kinesis';

cfg.poll_ms   = 50;                          % [ms] KPA polling
cfg.N_read    = 20;                          % 점당 읽기 횟수
cfg.N_dark    = 40;

cfg.u_list    = -0.0025:0.0005:0.0025;       % [u] 11점, QPD에서 약 ±0.1 mm   % [수정] 작은 빔: 직선 구간만
cfg.u_safe    = 0.01;
cfg.t_settle  = 0.2;                         % [s]
cfg.t_step_ms = 20;                          % [ms]

cfg.alpha     = 0.9;                         % Hybrid 가중치
cfg.J_abs     = [40.9, 37.8];                % [mm/u] S3 모델 |J| (L = 212 mm, 22.5°), 실측 아님
                                             % L을 쟀으면: cfg.J_abs = 0.193*L*[1, cosd(22.5)];
cfg.min_sum_V     = 0.02;                    % [V] dark 제거 SUM 최소값
cfg.noise_sigma   = 5;                       % dark 잡음의 몇 배를 무효 하한으로
cfg.min_fit_points = 5;
cfg.min_slope     = 1e-6;
cfg.sum_warn_frac = 0.05;
cfg.delta_sign_verified = false;             % δ 부호를 실험으로 확인한 뒤에만 true

cfg.save_dir = fullfile(fileparts(mfilename('fullpath')), 'DATA');
if isempty(fileparts(mfilename('fullpath'))), cfg.save_dir = fullfile(pwd, 'DATA'); end
if ~isfolder(cfg.save_dir), mkdir(cfg.save_dir); end        % DATA 폴더 없으면 자동 생성

%% 2. 사전 검사
assert(isfile(cfg.ini_file), 'S46642 ini 파일을 찾을 수 없음: %s', cfg.ini_file);
assert(max(abs(cfg.u_list)) <= cfg.u_safe, '스캔 범위가 u_safe를 넘음');
assert(any(abs(cfg.u_list) < 1e-12), 'u_list에 u = 0이 없음');
needed = {'MTIDevice', 'estimator_dos', 'estimator_logratio', 'estimator_hybrid'};
for i = 1:numel(needed)
    if isempty(which(needed{i})), error('함수를 찾을 수 없음: %s', needed{i}); end
end
cfg.hybrid_nargin = nargin('estimator_hybrid');
assert(ismember(cfg.hybrid_nargin, [4 5]), 'estimator_hybrid 인수 개수가 예상과 다름: %d', cfg.hybrid_nargin);
fprintf('\n=== E3 사전 검사 ===\nini: %s\nHybrid 인수 개수: %d\n', cfg.ini_file, cfg.hybrid_nargin);
if exist('mMTIDevice', 'var') && ~isempty(mMTIDevice)
    error('mMTIDevice 연결이 남아 있음. 레이저 끄기 명령으로 연결 해제 후 다시 실행');
end

mti = []; kpa = []; scan_x = []; scan_y = []; dark = []; off = []; th = [];

%% 3. 장비 연결 및 측정
try
    % 3-1. MZ 연결 (S46642 ini)
    mti = MTIDevice;
    if isempty(cfg.com_port), mti.ConnectDevice(); else, mti.ConnectDevice(cfg.com_port); end
    check_mti(mti, 'ConnectDevice');
    mti.SetDeviceParams(mti.LoadDeviceParams(cfg.ini_file));
    check_mti(mti, 'SetDeviceParams');
    mti.SetDeviceParam(MTIParam.DataScale, 1.0);
    mti.SetDeviceParam(MTIParam.SyncMode, MTISync.Output_DOut0);
    mti.SetDeviceParam(MTIParam.DigitalOutputEnable, 1);
    check_mti(mti, 'MZ 기본 설정');
    bw    = mti.GetDeviceParam(MTIParam.HardwareFilterBw);
    vmax  = mti.GetDeviceParam(MTIParam.VdifferenceMax);
    vbias = mti.GetDeviceParam(MTIParam.Vbias);
    fprintf('MZ: 필터 %.1f Hz, VdiffMax %.1f V, Vbias %.1f V\n', bw, vmax, vbias);
    if abs(bw-120) > 1 || abs(vmax-173) > 1 || abs(vbias-90) > 1
        error('S46642 설정(120 Hz, 173 V, 90 V)이 적용되지 않음');
    end
    mti.SetDeviceParam(MTIParam.MEMSDriverEnable, 1);
    check_mti(mti, 'MEMSDriverEnable');
    mti.GoToDevicePosition(0, 0, 0, cfg.t_step_ms);          % 중심, 레이저 끔
    check_mti(mti, '원점 이동');

    % 3-2. KPA101 연결
    kpa = kpa_open(cfg.kinesis, cfg.poll_ms);

    % 3-3. dark (레이저 끔)
    pause(1);
    dark = kpa_read_n(kpa, cfg.N_dark, cfg.poll_ms);
    off.XDiff = mean(dark.XDiff, 'omitnan');
    off.YDiff = mean(dark.YDiff, 'omitnan');
    off.SUM   = mean(dark.SUM,   'omitnan');
    th = noise_thresholds(dark, cfg);
    fprintf('dark: XDiff %.5f, YDiff %.5f, SUM %.5f V\n', off.XDiff, off.YDiff, off.SUM);
    fprintf('무효 하한: SUM %.5f V, 반쪽 %.5f V\n', th.sum_min, th.half_min);

    % 3-4. 레이저 켜고 X, Y 스캔
    mti.GoToDevicePosition(0, 0, 255, cfg.t_step_ms);
    check_mti(mti, '레이저 켬');
    pause(1);
    fprintf('\n=== X축 스캔 ===\n');  scan_x = do_scan(mti, kpa, cfg, 1);
    fprintf('\n=== Y축 스캔 ===\n');  scan_y = do_scan(mti, kpa, cfg, 2);

    % 3-5. 원시 데이터 먼저 저장
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    raw_file = fullfile(cfg.save_dir, sprintf('E3_RAW_%s.mat', stamp));
    raw = struct('cfg', cfg, 'off', off, 'dark', dark, 'thresholds', th, ...
                 'scan_x', scan_x, 'scan_y', scan_y);
    save(raw_file, 'raw');
    fprintf('\n원시 데이터 저장: %s\n', raw_file);
catch ME
    if ~isempty(scan_x) || ~isempty(scan_y)
        try
            pf = fullfile(cfg.save_dir, sprintf('E3_PARTIAL_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
            save(pf, 'cfg', 'off', 'dark', 'scan_x', 'scan_y');
            fprintf(2, '부분 데이터 저장: %s\n', pf);
        catch
        end
    end
    close_devices(mti, kpa, cfg.t_step_ms);
    rethrow(ME);
end
close_devices(mti, kpa, cfg.t_step_ms);

%% 4. 분석
res_x = analyze_scan(scan_x, off, th, cfg, 1);
res_y = analyze_scan(scan_y, off, th, cfg, 2);
names = {'DoS', 'LogRatio', 'Hybrid'};
S = NaN(2, 2, 3);                    % 행: δx, δy / 열: ux, uy / 3번째: 추정기
k_model_mm = NaN(3, 2);              % 행: 추정기 / 열: X, Y
J_sign_inferred = NaN(3, 2);
for m = 1:3
    S(:,:,m) = [res_x.slope(:,m), res_y.slope(:,m)];
    main_slope = [S(1,1,m), S(2,2,m)];
    if any(~isfinite(main_slope)) || any(abs(main_slope) < cfg.min_slope)
        error('%s: 주축 감도가 유효하지 않음', names{m});
    end
    k_model_mm(m,:)      = cfg.J_abs ./ abs(main_slope);
    J_sign_inferred(m,:) = sign(main_slope);
end

%% 5. 결과 출력
fprintf('\n========== E3 결과 ==========\n');
fprintf('S: 실측 감도 [1/u],  k: S3 |J| 기반 환산 [mm]\n');
for m = 1:3
    fprintf('\n[%s]\n', names{m});
    fprintf('S = [ %+10.5f  %+10.5f\n      %+10.5f  %+10.5f ]\n', S(1,1,m), S(1,2,m), S(2,1,m), S(2,2,m));
    fprintf('kX = %.5f mm,  kY = %.5f mm\n', k_model_mm(m,1), k_model_mm(m,2));
    fprintf('J 부호 (추정) = [%+d, %+d]\n', J_sign_inferred(m,1), J_sign_inferred(m,2));
    fprintf('간섭: uY -> δX = %+.2f %%,  uX -> δY = %+.2f %%\n', ...
        100*S(1,2,m)/S(1,1,m), 100*S(2,1,m)/S(2,2,m));
    fprintf('u = 0 위치: X = %+.4f mm, Y = %+.4f mm\n', ...
        k_model_mm(m,1)*res_x.intercept(1,m), k_model_mm(m,2)*res_y.intercept(2,m));
    fprintf('잡음: X = %.2f um, Y = %.2f um\n', ...
        res_x.noise_delta(1,m)*k_model_mm(m,1)*1e3, res_y.noise_delta(2,m)*k_model_mm(m,2)*1e3);
    fprintf('히스테리시스: X = %.2f um, Y = %.2f um\n', ...
        res_x.hyst_delta(1,m)*k_model_mm(m,1)*1e3, res_y.hyst_delta(2,m)*k_model_mm(m,2)*1e3);
end
fprintf('\nSUM 변화: X 스캔 %.1f %%, Y 스캔 %.1f %%\n', res_x.sum_var*100, res_y.sum_var*100);
if ~cfg.delta_sign_verified
    warning('δ 부호가 실험으로 확인되지 않음. J 부호는 조건부 추정값');
end

%% 6. 저장
cal = struct('cfg', cfg, 'off', off, 'dark', dark, 'thresholds', th, ...
             'scan_x', scan_x, 'scan_y', scan_y, 'res_x', res_x, 'res_y', res_y);
cal.names = names;
cal.S_delta_per_u   = S;
cal.k_model_mm      = k_model_mm;
cal.J_sign_inferred = J_sign_inferred;
cal.J_model_abs_mm_per_u = cfg.J_abs;
cal.raw_file = raw_file;
cal.time = datetime('now');
out_file = fullfile(cfg.save_dir, sprintf('E3_CAL_%s.mat', stamp));
save(out_file, 'cal');
fprintf('\n결과 저장: %s\n', out_file);

%% 7. 그림
plot_scan(res_x, k_model_mm(:,1), names);
plot_scan(res_y, k_model_mm(:,2), names);
fprintf('\n=== E3 완료 ===\n');


%% ==================== local functions ====================
function check_mti(mti, what)
    err = mti.GetLastError();
    if err ~= MTIError.MTI_SUCCESS
        error('Mirrorcle 오류: %s / code = %g', what, double(err));
    end
end

function scan = do_scan(mti, kpa, cfg, ax)
    u_seq = [cfg.u_list, fliplr(cfg.u_list)];
    n = numel(u_seq);
    scan.ax  = ax;
    scan.u   = u_seq(:);
    scan.dir = [ones(1, numel(cfg.u_list)), -ones(1, numel(cfg.u_list))]';
    scan.XDiff = NaN(n, cfg.N_read);  scan.YDiff = NaN(n, cfg.N_read);  scan.SUM = NaN(n, cfg.N_read);
    axname = 'XY';
    go_to(mti, ax, u_seq(1), cfg);  pause(0.5);
    for i = 1:n
        go_to(mti, ax, u_seq(i), cfg);
        pause(cfg.t_settle);
        d = kpa_read_n(kpa, cfg.N_read, cfg.poll_ms);
        scan.XDiff(i,:) = d.XDiff;  scan.YDiff(i,:) = d.YDiff;  scan.SUM(i,:) = d.SUM;
        fprintf('%s %2d/%d | u = %+.4f | SUM = %.4f V\n', axname(ax), i, n, u_seq(i), mean(d.SUM, 'omitnan'));
    end
    go_to(mti, ax, 0, cfg);
end

function go_to(mti, ax, u, cfg)
    if abs(u) > cfg.u_safe, error('명령이 안전 범위를 넘음'); end
    if ax == 1
        mti.GoToDevicePosition(u, 0, 255, cfg.t_step_ms);
    else
        mti.GoToDevicePosition(0, u, 255, cfg.t_step_ms);
    end
    check_mti(mti, 'GoToDevicePosition');
end

function th = noise_thresholds(dark, cfg)
    ds = dark.SUM;  dx = dark.XDiff;  dy = dark.YDiff;
    halves = [(ds+dx)/2; (ds-dx)/2; (ds+dy)/2; (ds-dy)/2];
    th.sum_min  = max(cfg.min_sum_V, cfg.noise_sigma*std(ds, 0, 'omitnan'));
    th.half_min = max(1e-9, cfg.noise_sigma*max(std(halves, 0, 2, 'omitnan')));
end

function R = analyze_scan(scan, off, th, cfg, ax)
    sig.XDiff = scan.XDiff - off.XDiff;
    sig.YDiff = scan.YDiff - off.YDiff;
    sig.SUM   = scan.SUM   - off.SUM;

    [~, ~, d1x, d1y, v1] = estimator_dos(sig, 1, th.sum_min);
    [~, ~, d2x, d2y, v2] = estimator_logratio(sig, 1, th.half_min);
    if cfg.hybrid_nargin == 4
        [d3x, d3y] = estimator_hybrid(sig, cfg.alpha, th.half_min, th.sum_min);
    else
        [d3x, d3y] = estimator_hybrid(sig, 1, cfg.alpha, th.half_min, th.sum_min);
    end
    v3 = v1 & v2 & isfinite(d3x) & isfinite(d3y);
    D = {d1x, d2x, d3x; d1y, d2y, d3y};
    V = {v1, v2, v3};

    u = scan.u;  axname = 'XY';
    R.ax = ax;  R.axis = axname(ax);  R.u = u;  R.dir = scan.dir;
    R.slope = NaN(2,3);  R.intercept = NaN(2,3);  R.p = NaN(2,2,3);
    R.mean_delta = NaN(numel(u),2,3);
    R.resid_delta = NaN(2,3);  R.noise_delta = NaN(2,3);  R.hyst_delta = NaN(2,3);
    R.sum_mean = mean(sig.SUM, 2, 'omitnan');
    if any(~isfinite(R.sum_mean)) || mean(R.sum_mean) <= 0
        error('%s축 SUM이 유효하지 않음', R.axis);
    end
    R.sum_var = (max(R.sum_mean) - min(R.sum_mean)) / mean(R.sum_mean);

    for m = 1:3
        valid = V{m} & isfinite(sig.SUM) & isfinite(sig.XDiff) & isfinite(sig.YDiff);
        for comp = 1:2
            d = D{comp,m};  d(~valid) = NaN;
            dm = mean(d, 2, 'omitnan');
            good = isfinite(dm);
            if numel(unique(u(good))) < cfg.min_fit_points
                error('%s축 / 추정기 %d / 성분 %d: 유효 점 부족', R.axis, m, comp);
            end
            p = polyfit(u(good), dm(good), 1);
            R.p(comp,:,m) = p;  R.slope(comp,m) = p(1);  R.intercept(comp,m) = p(2);
            R.mean_delta(:,comp,m) = dm;
            R.resid_delta(comp,m) = sqrt(mean((dm(good) - polyval(p, u(good))).^2));
            c0 = abs(u) < 1e-12;
            R.noise_delta(comp,m) = mean(std(d(c0,:), 0, 2, 'omitnan'), 'omitnan');
            up = dm(scan.dir == 1);  dn = flipud(dm(scan.dir == -1));
            dp = abs(up - dn);  dp = dp(isfinite(dp));
            if ~isempty(dp), R.hyst_delta(comp,m) = max(dp); end
        end
    end
end

function plot_scan(R, k_axis, names)
    figure('Name', sprintf('E3 %s축', R.axis), 'Position', [80 80 1200 680]);
    for m = 1:3
        dm = R.mean_delta(:, R.ax, m);
        p  = squeeze(R.p(R.ax, :, m));
        subplot(2,3,m); hold on; grid on;
        plot(R.u(R.dir == 1),  dm(R.dir == 1),  'o');
        plot(R.u(R.dir == -1), dm(R.dir == -1), 's');
        plot(R.u, polyval(p, R.u), 'k-');
        xlabel(sprintf('u_%s', lower(R.axis))); ylabel('\delta [-]');
        title(sprintf('%s: k = %.4f mm', names{m}, k_axis(m)));
        if m == 1, legend('올라감', '내려감', '직선', 'Location', 'best'); end
        subplot(2,3,m+3); grid on;
        plot(R.u, (dm - polyval(p, R.u))*k_axis(m)*1e3, 'o');
        xlabel(sprintf('u_%s', lower(R.axis))); ylabel('잔차 [um]');
    end
end

function kpa = kpa_open(kinesis_dir, poll_ms)
    NET.addAssembly(fullfile(kinesis_dir, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
    asm = NET.addAssembly(fullfile(kinesis_dir, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
    Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
    lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
          Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
    if lst.Count == 0, error('KPA101을 찾을 수 없음'); end
    sn  = char(lst.Item(0));
    dev = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
    kpa.dev = dev;
    try
        dev.Connect(sn);
        dev.WaitForSettingsInitialized(10000);
        dev.StartPolling(poll_ms);  pause(0.5);
        dev.EnableDevice();         pause(0.5);
        mt = asm.AssemblyHandle.GetType( ...
             'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
        dev.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false);  pause(0.5);
        fprintf('KPA101 연결: %s (Monitor)\n', sn);
    catch ME
        kpa_close(kpa);
        rethrow(ME);
    end
end

function d = kpa_read_n(kpa, N, poll_ms)
    d.XDiff = NaN(1,N);  d.YDiff = NaN(1,N);  d.SUM = NaN(1,N);
    for i = 1:N
        st = kpa.dev.Status;
        d.XDiff(i) = System.Convert.ToDouble(st.PositionDifference.X);
        d.YDiff(i) = System.Convert.ToDouble(st.PositionDifference.Y);
        d.SUM(i)   = System.Convert.ToDouble(st.Sum);
        pause(poll_ms/1000);
    end
end

function kpa_close(kpa)
    if isempty(kpa), return; end
    try kpa.dev.StopPolling(); catch ME, warning('KPA StopPolling 실패: %s', ME.message); end
    try kpa.dev.Disconnect(true); catch ME, warning('KPA Disconnect 실패: %s', ME.message); end
end

function close_devices(mti, kpa, t_step_ms)
    % 단계마다 따로 시도: 하나가 실패해도 나머지 종료 단계는 계속 실행
    if ~isempty(mti)
        try mti.GoToDevicePosition(0, 0, 0, t_step_ms); catch ME, warning('레이저 끄기 실패: %s', ME.message); end
    end
    kpa_close(kpa);
    if ~isempty(mti)
        try mti.ResetDevicePosition(); catch ME, warning('원점 복귀 실패: %s', ME.message); end
        try mti.SetDeviceParam(MTIParam.MEMSDriverEnable, 0); catch ME, warning('드라이버 OFF 실패: %s', ME.message); end
        try mti.DisconnectDevice(); catch ME, warning('연결 해제 실패: %s', ME.message); end
        try delete(mti); catch ME, warning('객체 삭제 실패: %s', ME.message); end
    end
end