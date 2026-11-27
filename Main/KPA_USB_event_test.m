function R = KPA_USB_event_test()
% KPA101 status-event diagnostic; no FSM commands or registry changes.
% Close Kinesis GUI and disconnect the prior KPA MATLAB session first.
% Output event times are MATLAB callback times, NOT hardware timestamps.
kin = 'C:\Program Files\Thorlabs\Kinesis';
NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
assert(lst.Count == 1,'Connect exactly one KPA101.');
sn = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);
clean = onCleanup(@() close_device(kpa)); %#ok<NASGU>
kpa.WaitForSettingsInitialized(10000);
kpa.StartPolling(20); pause(0.5);
kpa.EnableDevice(); pause(0.5);
mt = asm.AssemblyHandle.GetType('Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
kpa.SetOperatingMode(System.Enum.Parse(mt,'Monitor'),false); pause(0.5);
kpa.StopPolling(); pause(0.3);

N = 100000; ts = NaN(N,1); n = 0; active = false; clock0 = tic;
try
    listenerObj = addlistener(kpa,'StatusUpdate',@on_status);
catch ME
    fprintf('StatusUpdate listener failed: %s\n',ME.message);
    disp('Installed method signatures:'); disp(methods(kpa,'-full'));
    R = table(); return
end
listenerClean = onCleanup(@() delete_listener(listenerObj)); %#ok<NASGU>

polls = [1 2 5 10 20]; T = 5;
M = NaN(numel(polls),8); raw = cell(numel(polls),1);
for j = 1:numel(polls)
    active = false; kpa.StopPolling(); pause(0.3);
    kpa.StartPolling(polls(j)); pause(0.5);
    duration = NaN;
    try duration = double(kpa.PollingDuration()); catch
        try duration = double(kpa.PollingDuration); catch, end
    end
    n = 0; clock0 = tic; active = true;
    r = NaN(N,3); tr = NaN(N,1); nr = 0;
    while toc(clock0) < T && nr < N
        nr = nr+1;
        st = kpa.Status;
        r(nr,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
                   System.Convert.ToDouble(st.PositionDifference.Y), ...
                   System.Convert.ToDouble(st.Sum)];
        tr(nr) = toc(clock0);
        pause(0.001); % let MATLAB process callbacks
    end
    elapsed = toc(clock0); active = false;
    te = ts(1:n); r = r(1:nr,:); tr = tr(1:nr);
    changes = [false; any(diff(r,1,1) ~= 0,2)];
    [em,ep] = interval_stats(te); [cm,~] = interval_stats(tr(changes));
    M(j,:) = [polls(j),duration,n,n/elapsed,em,ep,sum(changes),cm];
    raw{j} = struct('event_t',te,'read_t',tr,'raw',r,'changed',changes,'elapsed',elapsed);
    fprintf('Poll %g ms | events %d | %.1f Hz | event median %.3f ms | changes %d\n', ...
        polls(j),n,n/elapsed,em,sum(changes));
end
R = array2table(M,'VariableNames',{'poll_ms','reported_poll_ms','event_count','event_Hz', ...
    'event_median_ms','event_p90_ms','change_count','change_median_ms'});
disp(R);

% Check for unsolicited callbacks with polling stopped before testing direct requests.
active = false; kpa.StopPolling(); pause(0.5);
n = 0; clock0 = tic; active = true; pause(1); active = false;
idle_events = n; fprintf('Events with polling stopped (1 s): %d\n',idle_events);
req_ms = NaN(30,1); req_error = '';
if idle_events == 0
    fprintf('Testing RequestStatus(), at most one outstanding request (stop on timeout).\n');
    n = 0; clock0 = tic; active = true;
    for i = 1:numel(req_ms)
        before = n; t_req = toc(clock0);
        try kpa.RequestStatus(); catch ME
            req_error = ME.message; fprintf('RequestStatus failed: %s\n',req_error); break
        end
        while n == before && toc(clock0)-t_req < 0.5
            pause(0.001);
        end
        if n == before
            fprintf('No StatusUpdate after request %d (500 ms timeout).\n',i); break
        end
        req_ms(i) = 1000*(ts(before+1)-t_req);
        pause(0.005);
    end
    active = false;
    valid = req_ms(isfinite(req_ms));
    if ~isempty(valid), fprintf('Request-to-next-event: %d/%d, median %.3f ms\n',numel(valid),numel(req_ms),median(valid)); end
else
    fprintf('Direct request test skipped: background events cannot be distinguished.\n');
end
file = fullfile(pwd,['KPA_event_diag_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(file,'R','raw','idle_events','req_ms','req_error','sn'); fprintf('Saved: %s\n',file);
try delete(listenerObj); catch, end            % [수정] 리스너 먼저 삭제
try kpa.StopPolling(); catch, end; pause(0.5)  % [수정] 폴링 멈춘 뒤
try kpa.Disconnect(true); catch, end           % [수정] 마지막에 연결 끊기

    function on_status(~,~)
        if active && n < N
            n = n+1; ts(n) = toc(clock0);
        end
    end
end

function [m,p] = interval_stats(t)
d = 1000*diff(t); m = NaN; p = NaN;
if ~isempty(d)
    m = median(d); d = sort(d); p = d(max(1,ceil(0.9*numel(d))));
end
end
function delete_listener(h)
try delete(h); catch, end
end
function close_device(kpa)
try kpa.StopPolling(); catch, end
pause(0.25);
try kpa.Disconnect(true); catch, end
end