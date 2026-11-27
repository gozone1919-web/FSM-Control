function R = KPA_serial_test(port)
% KPA101을 Kinesis 없이 COM 포트(APT 프로토콜)로 직접 읽어서 갱신 속도를 재는 시험
%   미러 명령 없음. Kinesis로 KPA101에 연결하지 않은 상태에서 실행할 것 (동시 연결 불가)
%   사용법: R = KPA_serial_test("COM4")   % 포트 번호는 장치 관리자에서 확인
%   메시지 형식 참고: Thorlabs APT Communications Protocol, thorlabs_apt_device(파이썬, MIT)
%     측정값 요청  MGMSG_QUAD_REQ_PARAMS 0x0871, param1 = 3 (readings)
%     측정값 응답  MGMSG_QUAD_GET_PARAMS 0x0872, 데이터 = subID(u16) XDiff(i16) YDiff(i16) Sum(u16) XPos(i16) YPos(i16)
%     연속 전송    MGMSG_HW_START_UPDATEMSGS 0x0011 / STOP 0x0012, 상태 메시지 0x0881, 확인 0x0882
%   값은 장치 내부 정수(raw). 볼트 환산은 Kinesis 값과 비교해서 따로 정함
if nargin < 1
    disp(serialportlist("available"));
    error('포트를 넣어서 실행: KPA_serial_test("COM4")');
end
s = serialport(port, 115200, "DataBits", 8, "Parity", "none", "StopBits", 1, ...
    "FlowControl", "hardware", "Timeout", 0.5);
pause(0.05); flush(s); pause(0.05);
DEST = 0x50; SRC = 0x01;
hdr = @(id, p1) uint8([bitand(id,255), bitshift(id,-8), p1, 0, DEST, SRC]);

%% 1. 동작 모드 확인 (0x0871, param1 = 7)
write(s, hdr(0x0871, 7), "uint8");
[id, data] = read_msg(s);
if id == 0x0872 && numel(data) >= 4
    d = uint8(data); mode = typecast(d(3:4), 'uint16');
    fprintf('동작 모드 응답: %d  (Monitor로 설정돼 있어야 함)\n', mode);
else
    fprintf('동작 모드 응답 없음 또는 형식 다름 (id 0x%04X, 데이터 %d바이트)\n', id, numel(data));
end

%% 2. 요청-응답 방식: 측정값을 요청하고 응답을 받을 때까지의 시간
N = 500; rtt = NaN(N,1); V = NaN(N,5); bad = 0; t_all = tic;
for i = 1:N
    t0 = tic;
    write(s, hdr(0x0871, 3), "uint8");
    [id, data] = read_msg(s);
    rtt(i) = 1000*toc(t0);
    if id == 0x0872 && numel(data) >= 12
        d = uint8(data);
        V(i,:) = double([typecast(d(3:4),'int16'), typecast(d(5:6),'int16'), ...
            typecast(d(7:8),'uint16'), typecast(d(9:10),'int16'), typecast(d(11:12),'int16')]);
    else
        bad = bad + 1;
        if id < 0, flush(s); end          % 시간 초과면 버퍼 비우고 계속
    end
end
T_all = toc(t_all);
ok = all(isfinite(V), 2);
chg = sum(any(diff(V(ok,1:3),1,1) ~= 0, 2));
fprintf('\n[요청-응답] %d회 중 성공 %d, 실패 %d\n', N, sum(ok), bad);
fprintf('  응답 시간: 중앙 %.2f ms, 90%% %.2f ms, 최대 %.2f ms\n', median(rtt(ok)), prctile(rtt(ok),90), max(rtt(ok)));
fprintf('  실제 반복 속도: %.1f Hz (%.2f ms마다 1개)\n', N/T_all, 1000*T_all/N);
fprintf('  값이 바뀐 횟수: %d / %d\n', chg, sum(ok)-1);
fprintf('  평균 raw 값: XDiff %.1f, YDiff %.1f, Sum %.1f\n', mean(V(ok,1)), mean(V(ok,2)), mean(V(ok,3)));

%% 3. 연속 전송 방식: 장치가 알아서 보내는 상태 메시지 간격
flush(s);
write(s, hdr(0x0011, 0), "uint8");
t0 = tic; last_ack = 0; tm = []; ids = [];
while toc(t0) < 3
    if s.NumBytesAvailable >= 6
        [id, ~] = read_msg(s);
        tm(end+1) = toc(t0); ids(end+1) = id;                %#ok<AGROW>
    else
        pause(0.0005);
    end
    if toc(t0) - last_ack > 0.5
        write(s, hdr(0x0882, 0), "uint8"); last_ack = toc(t0);  % 확인 메시지 (안 보내면 멈출 수 있음)
    end
end
write(s, hdr(0x0012, 0), "uint8"); pause(0.1); flush(s);
u = unique(ids);
fprintf('\n[연속 전송] 3초 동안 받은 메시지\n');
for k = 1:numel(u)
    tk = tm(ids == u(k));
    if numel(tk) > 1, dm = 1000*median(diff(tk)); else, dm = NaN; end
    fprintf('  id 0x%04X: %d개, 간격 중앙 %.2f ms\n', u(k), numel(tk), dm);
end
if isempty(u), fprintf('  받은 메시지 없음\n'); end

R = struct('rtt_ms', rtt, 'raw', V, 'bad', bad, 'T_all', T_all, 'stream_t', tm, 'stream_id', ids);
file = fullfile(pwd, ['KPA_serial_test_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(file, 'R'); fprintf('\n저장: %s\n', file);
clear s                                                   % 포트 닫기
end

function [id, data] = read_msg(s)
% APT 메시지 하나 읽기. 헤더 6바이트, 5번째 바이트 최상위 비트가 1이면 데이터가 뒤따름
data = [];
h = read(s, 6, "uint8");
if numel(h) < 6, id = -1; return, end
id = h(1) + 256*h(2);
if bitand(h(5), 0x80)
    len = h(3) + 256*h(4);
    data = read(s, len, "uint8");
end
end