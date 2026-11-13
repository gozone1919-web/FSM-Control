%% laser_on.m
% 정렬용: S46642 설정 -> 미러 중심(u = 0) -> 레이저 켬
% 끝나면 laser_off 실행 (calibration 등 다른 프로그램 실행 전에 반드시)

if exist('mMTIDevice', 'var') && ~isempty(mMTIDevice)
    disp('이미 연결돼 있음. 레이저만 켭니다.');
    mMTIDevice.GoToDevicePosition(0, 0, 255, 20);
    return;
end

mMTIDevice = MTIDevice;
mMTIDevice.ConnectDevice();
if mMTIDevice.GetLastError() ~= MTIError.MTI_SUCCESS
    delete(mMTIDevice); clear mMTIDevice
    error('MZ 컨트롤러 연결 실패 (다른 프로그램이 연결 중이면 끄고 다시)');
end
mMTIDevice.ResetDevicePosition();
mMTIDevice.SetDeviceParam(MTIParam.VdifferenceMax, 173);
mMTIDevice.SetDeviceParam(MTIParam.HardwareFilterBw, 120);
mMTIDevice.SetDeviceParam(MTIParam.Vbias, 90);
mMTIDevice.SetDeviceParam(MTIParam.DataScale, 1.0);
mMTIDevice.SetDeviceParam(MTIParam.SyncMode, MTISync.Output_DOut0);
mMTIDevice.SetDeviceParam(MTIParam.DigitalOutputEnable, 1);
mMTIDevice.SetDeviceParam(MTIParam.MEMSDriverEnable, 1);
mMTIDevice.GoToDevicePosition(0, 0, 255, 20);
disp('레이저 켜짐, 미러 중심 (u = 0). 끝나면 laser_off');