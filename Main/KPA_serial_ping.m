function KPA_serial_ping(port)
% KPA101 COM 포트 연결 확인. LED가 깜빡이면 명령이 장치에 닿은 것
for fc = ["hardware", "none"]
    fprintf('\n=== FlowControl = %s ===\n', fc);
    s = serialport(port, 115200, "DataBits", 8, "Parity", "none", "StopBits", 1, ...
        "FlowControl", fc, "Timeout", 0.3);
    try setRTS(s, true); catch, end
    pause(0.05); flush(s); pause(0.05);
    write(s, uint8([0x18 0x00 0x00 0x00 0x50 0x01]), "uint8");   % HW_NO_FLASH_PROGRAMMING
    pause(0.05);
    write(s, uint8([0x23 0x02 0x00 0x00 0x50 0x01]), "uint8");   % MOD_IDENTIFY: LED 깜빡임
    fprintf('LED가 깜빡였는지 보기 (2초)\n'); pause(2);
    flush(s);
    write(s, uint8([0x05 0x00 0x00 0x00 0x50 0x01]), "uint8");   % HW_REQ_INFO
    pause(0.3);
    nb = s.NumBytesAvailable;
    fprintf('HW_REQ_INFO 응답 바이트 수: %d (정상이면 90)\n', nb);
    if nb > 0
        b = read(s, min(nb, 30), "uint8");
        fprintf('앞부분: %s\n', sprintf('%02X ', b));
    end
    clear s; pause(0.3);
end
end