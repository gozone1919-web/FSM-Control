function KPA_serial_ping2(port)
% KPA101 COM 연결 확인 + 상태 갱신 요청(0x0880) 응답 속도 측정. 미러 명령 없음
for fc = ["none", "hardware"]
    fprintf('\n=== FlowControl = %s ===\n', fc);
    s = serialport(port, 115200, "DataBits", 8, "Parity", "none", "StopBits", 1, ...
        "FlowControl", fc, "Timeout", 0.2);
    try setRTS(s, true); catch, end
    pause(0.05); flush(s); pause(0.05);

    write(s, uint8([0x23 0x02 0x00 0x00 0x50 0x01]), "uint8");   % MOD_IDENTIFY: LED 깜빡임
    pause(1); flush(s);

    write(s, uint8([0x05 0x00 0x00 0x00 0x50 0x01]), "uint8");   % HW_REQ_INFO
    pause(0.3);
    fprintf('HW_REQ_INFO 응답 바이트: %d (정상이면 90)\n', s.NumBytesAvailable);
    flush(s);

    % 상태 갱신 요청 0x0880 → 응답 0x0881 (헤더 6 + 데이터 14 = 20바이트 예상)
    N = 200; rtt = NaN(N,1); got = zeros(N,1); V = NaN(N,3);
    for i = 1:N
        t0 = tic;
        write(s, uint8([0x80 0x08 0x00 0x00 0x50 0x01]), "uint8");
        while s.NumBytesAvailable < 20 && toc(t0) < 0.2, end
        rtt(i) = 1000*toc(t0); got(i) = s.NumBytesAvailable;
        if got(i) >= 20
            b = uint8(read(s, got(i), "uint8"));
            if b(1) == 0x81 && b(2) == 0x08
                V(i,:) = double([typecast(b(7:8),'int16'), typecast(b(9:10),'int16'), typecast(b(11:12),'uint16')]);
            end
        else
            flush(s);
        end
    end
    ok = all(isfinite(V), 2);
    fprintf('상태 요청 %d회 중 정상 응답 %d\n', N, sum(ok));
    if any(ok)
        fprintf('  응답 시간: 중앙 %.2f ms, 90%% %.2f ms, 최대 %.2f ms\n', ...
            median(rtt(ok)), prctile(rtt(ok),90), max(rtt(ok)));
        fprintf('  평균 raw: XDiff %.1f, YDiff %.1f, Sum %.1f\n', mean(V(ok,1)), mean(V(ok,2)), mean(V(ok,3)));
    else
        fprintf('  받은 바이트 수 예시: %s\n', mat2str(got(1:min(10,N))'));
    end
    write(s, uint8([0x82 0x08 0x00 0x00 0x50 0x01]), "uint8");   % ACK_STATUSUPDATE
    clear s; pause(0.3);
end
end