%% noise_1_cap.m  잡음 1단계: 레이저 끔 + 캡 닫음 -> 순수 암전류
% 전제: Kinesis 꺼짐. 미러가 연결돼 있으면(mMTIDevice) 레이저를 끄고 시작, 아니면 바로 시작
if exist('mMTIDevice', 'var') && ~isempty(mMTIDevice)
    mMTIDevice.GoToDevicePosition(0, 0, 0, 20);  pause(0.5);   % 레이저 끔
end
input('QPD 캡을 닫고 Enter ', 's');
[D, t] = KPA_record(10);
out = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'DATA', ['DARK_CAP_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);
if mean(D(:,3)) > 0.2, warning('SUM %.3f V: 레이저가 켜져 있었을 수 있음. 레이저 끄고 다시 측정할 것', mean(D(:,3))); end
save(out, 'D', 't');  fprintf('저장: %s\n', out);