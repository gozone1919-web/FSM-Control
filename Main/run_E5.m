%% run_E5.m
% 추정기마다 MATLAB을 새로 띄워 NEW_TEST_closed_loop 실행 (실험 MATLAB이 꺼져도 이 창은 유지)
%   오늘 날짜 PART 파일이 없는 조합만 실행, 최대 3바퀴. 홀수 seed 1→5, 짝수 seed 5→1
%   주의: 이 MATLAB에서는 laser_on 등으로 장치(MZ, KPA)를 연결하지 말 것
root  = 'C:\Users\Geochang\OneDrive\Documents\MATLAB\FSM Control';
data  = fullfile(root, 'Main', 'DATA');
day   = '20261127';                                   % 날짜가 바뀌면 수정
exe   = fullfile(matlabroot, 'bin', 'matlab.exe');
seeds = 1:7;  ests = 1:4;                             % [수정 11/27] v8: seed 1~7, 추정기 4개
has = @(s, e) ~isempty(dir(fullfile(data, sprintf('E5v8_PART_seed%d_est%d_%s_*.mat', s, e, day))));   % [수정 11/27]

for pass = 1:3
    for s = seeds
        if mod(s, 2) == 1, order = ests; else, order = fliplr(ests); end
        for e = order
            if has(s, e), continue; end
            fprintf('[%d바퀴] seed %d est %d 시작 %s\n', pass, s, e, datestr(now, 'HH:MM:SS'));
            cmd = sprintf('"%s" -wait -batch "ovr_seed=%d; ovr_est=%d; cd(''%s''); NEW_TEST_closed_loop"', ...
                          exe, s, e, fullfile(root, 'Control'));
            st = system(cmd);
            fprintf('    끝 %s (종료 코드 %d, %s)\n', datestr(now, 'HH:MM:SS'), st, ...
                    ifelse_str(has(s, e), '저장됨', '실패 → 다음 바퀴에 다시'));
            pause(5);
        end
    end
end

fprintf('\n===== 누락 =====\n');  n = 0;
for s = seeds, for e = ests
    if ~has(s, e), fprintf('seed %d est %d\n', s, e); n = n + 1; end
end, end
fprintf('누락 %d개, 끝 %s\n', n, datestr(now, 'HH:MM:SS'));

function out = ifelse_str(c, a, b)
    if c, out = a; else, out = b; end
end