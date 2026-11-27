function R = KPA_USB_diagnostic()
% Read-only measurement diagnostic (no FSM commands). Close Kinesis GUI
% and finish/disconnect any existing experiment before running.
% Keep the laser on and the beam safely within the QPD tracking region.
kin = 'C:\Program Files\Thorlabs\Kinesis';
NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.DeviceManagerCLI.dll'));
asm = NET.addAssembly(fullfile(kin,'Thorlabs.MotionControl.KCube.PositionAlignerCLI.dll'));
Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.BuildDeviceList();
lst = Thorlabs.MotionControl.DeviceManagerCLI.DeviceManagerCLI.GetDeviceList( ...
    Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.DevicePrefix);
assert(lst.Count == 1, 'Connect exactly one KPA101 for this diagnostic.');
sn = char(lst.Item(0));
kpa = Thorlabs.MotionControl.KCube.PositionAlignerCLI.KCubePositionAligner.CreateKCubePositionAligner(sn);
kpa.Connect(sn);
cleanup = onCleanup(@() close_kpa(kpa)); %#ok<NASGU>
kpa.WaitForSettingsInitialized(10000);
kpa.StartPolling(20); pause(0.5);
kpa.EnableDevice(); pause(0.5);
mt = asm.AssemblyHandle.GetType('Thorlabs.MotionControl.KCube.PositionAlignerCLI.PositionAlignerStatus+OperatingModes');
kpa.SetOperatingMode(System.Enum.Parse(mt,'Monitor'),false); pause(0.5);
% Inspect actual installed API without calling unverified methods.
api_methods = string(methods(kpa));
disp('Potential status/request methods:');
disp(api_methods(contains(lower(api_methods),["request","status","poll","read","update"])));
try
    disp('Available events:'); disp(events(kpa));
catch
end
polls = [1 2 5 10 20]; T = 5; N = 100000;
M = NaN(numel(polls),7); raw = cell(numel(polls),1);
for j = 1:numel(polls)
    kpa.StopPolling(); pause(0.25);
    kpa.StartPolling(polls(j)); pause(0.5);
    t = NaN(N,1); r = NaN(N,3); read_ms = NaN(N,1);
    n = 0; t0 = tic;
    while toc(t0) < T && n < N
        pause(0.001);
        n = n+1; ta = toc(t0);
        st = kpa.Status;
        r(n,:) = [System.Convert.ToDouble(st.PositionDifference.X), ...
                  System.Convert.ToDouble(st.PositionDifference.Y), ...
                  System.Convert.ToDouble(st.Sum)];
        t(n) = toc(t0); read_ms(n) = 1000*(t(n)-ta);
    end
    t = t(1:n); r = r(1:n,:); read_ms = read_ms(1:n);
    changed = [false; any(diff(r,1,1) ~= 0,2)];
    tc = t(changed); dt = 1000*diff(tc);
    med = NaN; p90 = NaN; hz = NaN;
    if ~isempty(dt)
        med = median(dt); s = sort(dt); p90 = s(max(1,ceil(0.9*numel(s))));
        hz = (numel(tc)-1)/(tc(end)-tc(1));
    end
    M(j,:) = [polls(j), 1000*median(diff(t)), median(read_ms), med, p90, hz, sum(changed)];
    raw{j} = struct('t',t,'r',r,'changed',changed,'read_ms',read_ms);
    fprintf('Poll %g ms: observed change median %.3f ms, observed rate %.1f Hz\n',polls(j),med,hz);
end
R = array2table(M,'VariableNames',{'poll_ms','read_interval_ms','status_call_ms','change_median_ms','change_p90_ms','observed_change_Hz','change_count'});
disp(R);
% changed measures numeric differences, not actual packet arrivals.
file = fullfile(pwd,['KPA_USB_diag_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(file,'R','raw','api_methods','sn'); fprintf('Saved: %s\n',file);
end
function close_kpa(kpa)
try kpa.StopPolling(); catch, end
pause(0.25);
try kpa.Disconnect(true); catch, end
end
