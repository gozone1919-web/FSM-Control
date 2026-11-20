%% noise_3_laser.m  잡음 3단계: 레이저 켬 + 미러 중심 고정 (PID 없음) -> 빛이 있을 때 센서 잡음
% 전제: Kinesis 꺼짐, laser_on 상태, 캡 엶, noise_2_open을 먼저 실행 (그 평균을 dark로 뺌)
%   저장 NOISE_*.mat: Raw = 원시값 - dark 평균, dt = 시뮬레이션 Ts (init_pid_sim 'measured'용)

data_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'DATA');
f = dir(fullfile(data_dir, 'DARK_OPEN_*.mat'));
if isempty(f), error('DARK_OPEN 파일 없음: noise_2_open 먼저 실행'); end
[~, i] = max([f.datenum]);  dk = load(fullfile(f(i).folder, f(i).name));
dark = mean(dk.D, 1);

try mMTIDevice.StopDataStream(); catch, end
mMTIDevice.SetDeviceParam(MTIParam.OutputOffsets, 0, 0);
mMTIDevice.GoToDevicePosition(0, 0, 255, 20);  pause(1.5);
[D, t] = KPA_record(10);
fprintf('정렬 기준 (11/13 Kinesis): XDiff -0.0025, YDiff +0.0037, SUM 0.8264 V\n');

% 위치 잡음 [um]: 추정기 δ를 최신 빔 스캔 중심 기울기(±0.3 mm 피팅)로 나눔
Raw = D - dark;
s.XDiff = Raw(:,1); s.YDiff = Raw(:,2); s.SUM = Raw(:,3);
[~, ~, d1x, d1y] = estimator_dos(s, 1);  [~, ~, d2x, d2y] = estimator_logratio(s, 1);
names = {'DoS', 'LogRatio', 'Hybrid a=0.9', 'Hybrid a=0.8'};  al = [1 0 0.9 0.8];
for ax = 'XY'
    g = dir(fullfile(data_dir, ['BEAM_SCAN_' ax '_*.mat']));  [~, i] = max([g.datenum]);
    B = load(fullfile(g(i).folder, g(i).name));
    Xs = B.X(:);  m = abs(Xs) <= 0.1;  Xs = Xs(m);  M = B.M(m, :);          % [수정] 작은 빔: 중심 ±0.1 mm, 길이 맞춤
    b.XDiff = M(:,1); b.YDiff = M(:,2); b.SUM = M(:,3);                     % [수정]
    [~, ~, e1x, e1y] = estimator_dos(b, 1);  [~, ~, e2x, e2y] = estimator_logratio(b, 1);
    if ax == 'X', e1 = e1x; e2 = e2x; else, e1 = e1y; e2 = e2y; end
    p1 = polyfit(Xs, e1(:), 1);  p2 = polyfit(Xs, e2(:), 1);                % [수정]
    slope.(ax) = [p1(1) p2(1)];                       % [DoS LogRatio] 중심 기울기 [1/mm]
end
fprintf('\n위치 잡음 [um]      X 1σ    Y 1σ   반경 rms\n');
for j = 1:4
    a = al(j);
    dX = a*d1x + (1-a)*d2x;  dY = a*d1y + (1-a)*d2y;
    if a == 1, dX = d1x; dY = d1y; end                % 0×NaN 방지
    sx = a*slope.X(1) + (1-a)*slope.X(2);  sy = a*slope.Y(1) + (1-a)*slope.Y(2);
    ex = 1e3*(dX - mean(dX))/sx;  ey = 1e3*(dY - mean(dY))/sy;
    fprintf('%-14s %7.2f %7.2f %8.2f\n', names{j}, std(ex), std(ey), sqrt(mean(ex.^2 + ey.^2)));
end
fprintf('(폐루프 외란 전 잡음, seed 1~3 평균: 반경 약 2.8~3.0 um)\n');

dt = 0.0158;  dt_meas = median(diff(t));
out = fullfile(data_dir, ['NOISE_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);
save(out, 'Raw', 'dt', 'dt_meas', 'D', 't', 'dark');  fprintf('저장: %s\n', out);