%% beam_size.m
% QPD 위 빔 반경 r0 추정 (X, Y 축 연속 스캔)
%   FSM으로 빔을 X, Y 방향으로 QPD를 가로질러 이동 -> δ 측정 -> erf 곡선 피팅
%   두 축 모두 같은 거리(mm) 범위로 스캔 (축마다 J가 달라서 u 범위는 따로 계산)            % [수정]
%   범위 ±3.8 mm: 외란 최대 3.7 mm와 광량 손실 시작 위치를 두 축 모두 덮도록                % [수정]
%   저장: <프로젝트 루트>\Main\DATA\BEAM_SCAN_X_날짜.mat, BEAM_SCAN_Y_날짜.mat
% 전제: Kinesis 앱 꺼짐. 레이저용 mMTIDevice가 있으면 그대로 사용

scan_axes = {'X', 'Y'};                  % 스캔할 축 (하나만 하려면 {'X'} 등)
J_xy    = [40.900, 37.800];              % [mm/u] E3 대각 성분 [J_x, J_y]                   % [수정]
scan_mm = 3.8;                           % [mm] 두 축 공통 스캔 범위 ±scan_mm                % [수정] 작은 빔 중심 촘촘 스캔
n_pts   = 77;                            % 스캔 점 개수 (간격 0.1 mm)   % [수정 11/27] 시간 단축, 피팅 범위 ±0.3 mm에 7점
N_read  = 30;                            % 점당 읽기 횟수 (표준편차용)
N_dark  = 50;                            % dark 읽기 횟수
poll_ms = 50;
kin     = 'C:\Program Files\Thorlabs\Kinesis';

%% 0. 저장 경로: 프로젝트 루트\Main\DATA (실행 방식과 무관)
save_dir = find_data_dir();
fprintf('저장 폴더: %s\n', save_dir);
stamp = datestr(now, 'yyyymmdd_HHMMSS');

%% 1. MZ (레이저, 미러)
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
try mMTIDevice.StopDataStream(); catch, end                                                  % [수정]
mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);     % 이전 폐루프 오프셋 제거       % [수정]

%% 2. KPA101 (Monitor 모드)
NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);

SCN = struct();
try
    kpa.WaitForSettingsInitialized(10000);
    kpa.StartPolling(poll_ms); pause(0.5);
    kpa.EnableDevice(); pause(0.5);
    mt = asm.AssemblyHandle.GetType( ...
        'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
    kpa.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false); pause(0.5);

    %% 3. dark (레이저 끔) - 한 번만 측정
    mMTIDevice.GoToDevicePosition(0, 0, 0, 20); pause(1);
    dk      = read_kpa(kpa, N_dark, poll_ms);
    off     = mean(dk, 1);                               % [XDiff YDiff SUM]
    off_std = std(dk, 0, 1);                             % dark 표준편차 (바닥 잡음)
    fprintf('dark 평균: XDiff %.4f, YDiff %.4f, SUM %.4f V\n', off);
    fprintf('dark 표준편차: XDiff %.5f, YDiff %.5f, SUM %.5f V\n', off_std);

    %% 4. 축별 스캔 (레이저 켬)
    for ia = 1:numel(scan_axes)
        ax  = scan_axes{ia};
        if strcmp(ax, 'X'), Jax = J_xy(1); else, Jax = J_xy(2); end                           % [수정]
        u_list = linspace(-scan_mm/Jax, scan_mm/Jax, n_pts);    % 같은 mm 범위가 되도록       % [수정]
        n   = numel(u_list);
        M   = NaN(n, 3);                                 % 점별 평균 (dark 제거)
        S   = NaN(n, 3);                                 % 점별 표준편차
        Raw = NaN(N_read, 3, n);                         % 원시 샘플 (dark 제거)
        fprintf('\n=== %s축 스캔 (±%.2f mm, u ±%.4f) ===\n', ax, scan_mm, max(u_list));     % [수정]
        go_to(mMTIDevice, ax, u_list(1)); pause(0.5);
        for i = 1:n
            go_to(mMTIDevice, ax, u_list(i));
            pause(0.2);
            D          = read_kpa(kpa, N_read, poll_ms) - off;
            Raw(:,:,i) = D;
            M(i,:)     = mean(D, 1);
            S(i,:)     = std(D, 0, 1);
            fprintf('%s %2d/%d  u = %+.4f  XDiff = %+.3f  YDiff = %+.3f  SUM = %.3f V\n', ...
                ax, i, n, u_list(i), M(i,1), M(i,2), M(i,3));
        end
        mMTIDevice.GoToDevicePosition(0, 0, 255, 20); pause(0.5);   % 중심 복귀
        SCN.(ax) = struct('M', M, 'S', S, 'Raw', Raw, 'u_list', u_list, 'J', Jax);           % [수정]
    end
catch ME
    try kpa.StopPolling(); kpa.Disconnect(true); catch, end
    rethrow(ME);
end
kpa.StopPolling(); kpa.Disconnect(true); clear kpa

%% 5. 축별 피팅, 그림, 저장
model = @(p, X) p(1)*erf(sqrt(2)*(X - p(2))/abs(p(3))) + p(4);
for ia = 1:numel(scan_axes)
    ax  = scan_axes{ia};
    M   = SCN.(ax).M;  S = SCN.(ax).S;  Raw = SCN.(ax).Raw;
    u_list = SCN.(ax).u_list;  J = SCN.(ax).J;                                               % [수정]
    if strcmp(ax, 'X')
        d = -M(:,1) ./ M(:,3);                           % DoS X (estimator_dos와 같은 부호)
    else
        d =  M(:,2) ./ M(:,3);                           % DoS Y
    end
    X  = J * u_list(:);                                  % [mm] 스캔 축 위치
    p  = fminsearch(@(p) sum((model(p, X) - d).^2), [1, 0, 2.0, 0]);
    r0 = abs(p(3));
    S0 = interp1(X, M(:,3), 0);                                                               % [수정]
    k1 = find(X < 0 & M(:,3) < 0.95*S0, 1, 'last');                                           % [수정]
    k2 = find(X > 0 & M(:,3) < 0.95*S0, 1, 'first');                                          % [수정]
    if isempty(k1), x1 = NaN; else, x1 = X(k1); end                                           % [수정]
    if isempty(k2), x2 = NaN; else, x2 = X(k2); end                                           % [수정]

    fprintf('\n[%s축] 빔 반경 r0 (1/e^2) = %.3f mm  ->  지름 약 %.2f mm\n', ax, r0, 2*r0);
    fprintf('[%s축] u = 0일 때 빔 중심 위치 = %+.3f mm\n', ax, p(2));
    fprintf('[%s축] SUM 최대 %.3f V, 최소 %.3f V\n', ax, max(M(:,3)), min(M(:,3)));
    fprintf('[%s축] SUM이 중심의 95%% 아래가 되는 위치: %.2f / %+.2f mm\n', ax, x1, x2);      % [수정]

    figure('Name', sprintf('Beam size scan (%s)', ax), 'Position', [100 100 1000 420]);
    subplot(1,2,1); hold on; grid on;
    plot(X, d, 'o'); Xf = linspace(min(X), max(X), 400);
    plot(Xf, model(p, Xf), 'k-');
    xlabel(sprintf('%s [mm] (J \\times u)', ax)); ylabel('\delta (DoS)');
    title(sprintf('%s축 erf 피팅: r_0 = %.3f mm', ax, r0));
    subplot(1,2,2); grid on; hold on;                                                         % [수정]
    plot(X, M(:,3), 'o-');
    yline(0.95*S0, 'r--', '중심 SUM의 95%');                                                  % [수정]
    xlabel(sprintf('%s [mm]', ax)); ylabel('SUM [V] (dark 제거)');
    title('SUM vs 위치 (가운데가 움푹하면 틈 손실)');

    scan_axis = ax;
    out_file = fullfile(save_dir, sprintf('BEAM_SCAN_%s_%s.mat', ax, stamp));
    save(out_file, 'X', 'M', 'S', 'Raw', 'd', 'off', 'off_std', 'u_list', ...
        'J', 'J_xy', 'scan_axis', 'scan_mm', 'N_read', 'p', 'r0');                            % [수정]
    fprintf('저장: %s\n', out_file);
end

%% local functions
function D = read_kpa(kpa, N, poll_ms)
D = NaN(N, 3);
for k = 1:N
    st = kpa.Status;
    D(k,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
        System.Convert.ToDouble(st.PositionDifference.Y), ...
        System.Convert.ToDouble(st.Sum)];
    pause(poll_ms/1000);
end
end

function go_to(dev, ax, u)
if strcmp(ax, 'X')
    dev.GoToDevicePosition(u, 0, 255, 20);
else
    dev.GoToDevicePosition(0, u, 255, 20);
end
end

function save_dir = find_data_dir()
% 파일 위치(없으면 현재 폴더)에서 위로 올라가며 'Main' 폴더가 있는 곳을 루트로 사용
d = fileparts(mfilename('fullpath'));
if isempty(d), d = pwd; end
while true
    if isfolder(fullfile(d, 'Main')), break; end
    [parent, name] = fileparts(d);
    if strcmpi(name, 'Main'), d = parent; break; end
    if isempty(parent) || strcmp(parent, d)
        error('프로젝트 루트(Main 폴더)를 찾지 못함. FSM Control 폴더 안에서 실행할 것');
    end
    d = parent;
end
save_dir = fullfile(d, 'Main', 'DATA');
if ~isfolder(save_dir), mkdir(save_dir); end
end
