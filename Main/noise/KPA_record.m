function [D, t] = kpa_record(T)
% KPA101(Monitor)에 연결해 T초 동안 일정 간격(dt_rec)으로 기록하고 연결 해제
%   D = [XDiff YDiff SUM] [V] (dark 포함 원시값), t = 기록 시각 [s]
%   dt_rec = 16 ms: KPA가 새 값을 내는 간격(약 16 ms)과 같게 맞춤
kin = 'C:\Program Files\Thorlabs\Kinesis';
NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin, 'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);
try
    kpa.WaitForSettingsInitialized(10000);
    kpa.StartPolling(1); pause(0.5);
    kpa.EnableDevice(); pause(0.5);
    mt = asm.AssemblyHandle.GetType( ...
        'Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
    kpa.SetOperatingMode(System.Enum.Parse(mt, 'Monitor'), false); pause(0.5);

    dt_rec = 0.016;
    N = ceil(T/dt_rec);  D = NaN(N, 3);  t = NaN(N, 1);  t0 = tic;
    for k = 1:N
        while toc(t0) < (k-1)*dt_rec, end          % 일정 간격 유지
        st = kpa.Status;
        D(k,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
                  System.Convert.ToDouble(st.PositionDifference.Y), ...
                  System.Convert.ToDouble(st.Sum)];
        t(k) = toc(t0);
    end
catch ME
    try kpa.StopPolling(); kpa.Disconnect(true); catch, end
    rethrow(ME);
end
kpa.StopPolling(); kpa.Disconnect(true);
fprintf('%d개 기록 (%.0f초)\n', N, T);
fprintf('평균 (값의 수준)       XDiff %+.5f, YDiff %+.5f, SUM %.5f V\n', mean(D, 1));
fprintf('1σ (평균 주변 흔들림)  XDiff %.5f, YDiff %.5f, SUM %.5f V\n', std(D, 0, 1));
end