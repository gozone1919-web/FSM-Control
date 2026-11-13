%% laser_off.m
% 레이저 끄고 미러 원위치, 연결 해제

if ~exist('mMTIDevice', 'var') || isempty(mMTIDevice)
    disp('연결된 mMTIDevice 없음');
    return;
end
try mMTIDevice.GoToDevicePosition(0, 0, 0, 20); catch, end
try mMTIDevice.ResetDevicePosition(); catch, end
try mMTIDevice.SetDeviceParam(MTIParam.MEMSDriverEnable, 0); catch, end
try mMTIDevice.DisconnectDevice(); catch, end
try delete(mMTIDevice); catch, end
clear mMTIDevice
disp('레이저 끔, 연결 해제');