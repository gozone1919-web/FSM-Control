function R = KPA_request_test()
% KPA101 직접 요청(RequestStatus) 안정성 시험. 미러 명령 없음, 레지스트리 변경 없음.
%   폴링을 끈 상태에서 요청 간격을 바꿔 가며, 요청마다 StatusUpdate 이벤트가 오는지와 응답 시간을 잼
%   응답이 안 와도 멈추지 않고 다음 요청으로 넘어감 (끊긴 뒤 다시 살아나는지 확인)
%   실행 전: MATLAB 재시작, Kinesis 앱 끔, laser_on으로 빔이 QPD 위에 있게
kin = 'C:\Program Files\Thorlabs\Kinesis';
NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
assert(lst.Count == 1,'KPA101을 하나만 연결할 것');
sn  = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);
kpa.WaitForSettingsInitialized(10000);
kpa.StartPolling(20); pause(0.5);
kpa.EnableDevice(); pause(0.5);
mt = asm.AssemblyHandle.GetType('Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
kpa.SetOperatingMode(System.Enum.Parse(mt,'Monitor'),false); pause(0.5);
kpa.StopPolling(); pause(0.5);                       % 폴링 끔: 이후 이벤트는 요청에 대한 응답뿐

N = 10000; ts = NaN(N,1); n = 0; clock0 = tic;
listenerObj = addlistener(kpa,'StatusUpdate',@on_status);

gaps    = [40 20 10 5];        % [ms] 요청 간격 (느린 것부터)
n_req   = 60;                  % 간격마다 요청 횟수
timeout = 0.1;                 % [s] 응답 대기 한계
M = NaN(numel(gaps),7); raw = cell(numel(gaps),1);
for j = 1:numel(gaps)
    pause(0.3);
    n = 0; clock0 = tic;
    resp_ms = NaN(n_req,1); val = NaN(n_req,3); err = '';
    for i = 1:n_req
        t_req = toc(clock0); before = n;
        try kpa.RequestStatus(); catch ME, err = ME.message; break, end
        while n == before && toc(clock0) - t_req < timeout
            pause(0.001);
        end
        if n > before
            resp_ms(i) = 1000*(ts(before+1) - t_req);
            st = kpa.Status;                                 % 응답 직후 값
            val(i,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
                System.Convert.ToDouble(st.PositionDifference.Y), ...
                System.Convert.ToDouble(st.Sum)];
        end
        while toc(clock0) - t_req < gaps(j)/1000            % 다음 요청까지 간격 맞춤
            pause(0.001);
        end
    end
    ok = isfinite(resp_ms);
    first_fail = find(~ok, 1); if isempty(first_fail), first_fail = NaN; end
    recovered = any(ok((min(first_fail, n_req)+1):end));     % 끊긴 뒤 다시 응답이 왔는지
    vchg = sum(any(diff(val(ok,:),1,1) ~= 0, 2));            % 응답 사이 값이 바뀐 횟수
    M(j,:) = [gaps(j), sum(ok), n_req, median(resp_ms(ok)), max(resp_ms(ok)), first_fail, recovered];
    raw{j} = struct('resp_ms',resp_ms,'val',val,'err',err,'val_changes',vchg);
    fprintf('간격 %2d ms | 응답 %2d/%d | 응답시간 중앙 %.2f ms, 최대 %.2f ms | 첫 실패 %g | 실패 후 회복 %d | 값 변화 %d\n', ...
        gaps(j), sum(ok), n_req, median(resp_ms(ok)), max(resp_ms(ok)), first_fail, recovered, vchg);
    if ~isempty(err), fprintf('   RequestStatus 에러: %s\n', err); end
end
R = array2table(M,'VariableNames',{'gap_ms','ok','n_req','resp_median_ms','resp_max_ms','first_fail','recovered'});
disp(R);

file = fullfile(pwd,['KPA_request_test_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(file,'R','raw','sn'); fprintf('저장: %s\n',file);

% 정리 순서: 리스너 → 폴링 → 연결
try delete(listenerObj); catch, end
try kpa.StopPolling(); catch, end; pause(0.5)
try kpa.Disconnect(true); catch, end

    function on_status(~,~)
        if n < N, n = n+1; ts(n) = toc(clock0); end
    end
end