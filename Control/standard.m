%% tune_standard.m
% 기준 PID 튜닝 (추정기 무관): 선형 센서(X̂ = X) + 작은 랜덤 외란
%   P -> D -> I 순서로 한 항씩 정함
%   게인 단위: 루프 이득 (실제 블록 값 = 값 / J), Kd 단위 [s]

init_pid_sim;                       % Ts, FSM, J, 잡음 등 설정 (실험 E2 값)
mdl = 'PID_laptop';

%% 1. 설정
Kp_list = [0.1 0.2 0.3 0.4 0.5 0.7 0.9];    % 1단계
Kd_list = [0 0.0005 0.001 0.002 0.004];     % 2단계 [s]
Ki_list = [5 10 15 20 25 30 40 50 60];      % 3단계 [1/s]
N_d     = 30;                               % D 필터 계수 (N*Ts < 1)
seeds   = 1:5;
r_small = [0.2 0.5];                        % [mm] 작은 외란
os_max  = 15;                               % [%] 오버슈트 허용 한계
band    = 0.02;                             % 정착 밴드 (외란 크기 대비)

%% 2. 선형 센서 (추정기 영향 제거), D 필터
Xhat_sensor = Xs_sensor;  Yhat_sensor = Ys_sensor;
N_x = N_d;  N_y = N_d;
fprintf('\n센서: 선형 (X̂ = X), 외란 %.1f~%.1f mm, seed %d개, N = %d (N*Ts = %.2f)\n', ...
    r_small, numel(seeds), N_d, N_d*Ts);

%% 3. 1단계: P (Ki = 0, Kd = 0)
P1 = run_stage(mdl, Kp_list, 0, 0, 'Kp', seeds, r_small, band, J, step_time);
ok = P1.fail == 0 & P1.os_pct <= os_max;
score = P1.reduce_pct;  score(~ok) = -Inf;
[~, b] = max(score);  Kp_sel = Kp_list(b);
fprintf('\n[1단계 P] Ki = 0, Kd = 0\n');  disp(P1)
fprintf('-> Kp = %.2f 선택 (오차 감소 %.0f %%, 오버슈트 %.1f %%)\n', Kp_sel, P1.reduce_pct(b), P1.os_pct(b));

%% 4. 2단계: D (Kp 고정, Ki = 0)
D1 = run_stage(mdl, Kp_sel, 0, Kd_list, 'Kd', seeds, r_small, band, J, step_time);
ok = D1.fail == 0 & D1.os_pct <= os_max;
st = D1.settle_ms;  st(~ok) = Inf;
[~, b] = min(st);  Kd_sel = Kd_list(b);
fprintf('\n[2단계 D] Kp = %.2f, Ki = 0\n', Kp_sel);  disp(D1)
fprintf('-> Kd = %.4f s 선택 (자리 잡는 시간 %.0f ms, 오버슈트 %.1f %%)\n', Kd_sel, D1.settle_ms(b), D1.os_pct(b));

%% 5. 3단계: I (Kp, Kd 고정)
I1 = run_stage(mdl, Kp_sel, Ki_list, Kd_sel, 'Ki', seeds, r_small, band, J, step_time);
ok = I1.fail == 0 & I1.os_pct <= os_max;
st = I1.settle_ms;  st(~ok) = Inf;
[~, b] = min(st);  Ki_sel = Ki_list(b);
fprintf('\n[3단계 I] Kp = %.2f, Kd = %.4f s\n', Kp_sel, Kd_sel);  disp(I1)
fprintf('-> Ki = %.0f /s 선택 (정착 %.0f ms, 오버슈트 %.1f %%)\n', Ki_sel, I1.settle_ms(b), I1.os_pct(b));

fprintf('\n=== 기준 PID (루프 이득): Kp = %.2f, Ki = %.0f /s, Kd = %.4f s, N = %d ===\n', ...
    Kp_sel, Ki_sel, Kd_sel, N_d);

save(fullfile(data_dir, sprintf('TUNE_STD_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'))), ...
    'P1', 'D1', 'I1', 'Kp_sel', 'Kd_sel', 'Ki_sel', 'N_d', 'seeds', 'r_small', 'Ts');

%% 6. 그림: 단계별 결과
figure('Name', 'Standard tuning P-D-I', 'Color', 'w', 'Position', [80 80 1300 380]);
subplot(1,3,1); yyaxis left;  plot(Kp_list, P1.reduce_pct, 'o-'); ylabel('오차 감소 [%]');
yyaxis right; plot(Kp_list, P1.os_pct, 's--'); ylabel('오버슈트 [%]'); yline(os_max, ':');
xline(Kp_sel, 'k--'); grid on; xlabel('Kp'); title('1단계 P (Ki = 0, Kd = 0)');
subplot(1,3,2); yyaxis left;  plot(1e3*Kd_list, D1.settle_ms, 'o-'); ylabel('자리 잡는 시간 [ms]');
yyaxis right; plot(1e3*Kd_list, D1.os_pct, 's--'); ylabel('오버슈트 [%]'); yline(os_max, ':');
xline(1e3*Kd_sel, 'k--'); grid on; xlabel('Kd [ms]'); title(sprintf('2단계 D (Kp = %.2f)', Kp_sel));
subplot(1,3,3); yyaxis left;  plot(Ki_list, I1.settle_ms, 'o-'); ylabel('정착 시간 [ms]');
yyaxis right; plot(Ki_list, I1.os_pct, 's--'); ylabel('오버슈트 [%]'); yline(os_max, ':');
xline(Ki_sel, 'k--'); grid on; xlabel('Ki [1/s]'); title('3단계 I (Kp, Kd 고정)');

%% local functions
function T = run_stage(mdl, Kp, Ki, Kd, which, seeds, r_small, band, J, step_time)
% 한 게인만 바꿔 가며 seed별로 시뮬레이션. 변수는 Simulink가 읽는 base 워크스페이스에 씀
    switch which
        case 'Kp', vals = Kp;
        case 'Kd', vals = Kd;
        case 'Ki', vals = Ki;
    end
    to_zero = strcmp(which, 'Ki');            % I가 있으면 0까지, 없으면 최종값 근처까지
    n = numel(vals);  nS = numel(seeds);
    ST = NaN(n, nS);  OS = NaN(n, nS);  RD = NaN(n, nS);
    for iv = 1:n
        kp = Kp(1); ki = Ki(1); kd = Kd(1);
        switch which
            case 'Kp', kp = vals(iv);
            case 'Kd', kd = vals(iv);
            case 'Ki', ki = vals(iv);
        end
        assignin('base', 'Kp_x', kp/J(1,1));  assignin('base', 'Kp_y', kp/J(2,2));
        assignin('base', 'Ki_x', ki/J(1,1));  assignin('base', 'Ki_y', ki/J(2,2));
        assignin('base', 'Kd_x', kd/J(1,1));  assignin('base', 'Kd_y', kd/J(2,2));
        for is = 1:nS
            rs = RandStream('mt19937ar', 'Seed', seeds(is));
            r  = r_small(1) + diff(r_small)*rand(rs);
            th = 2*pi*rand(rs);
            dvec = r*[cos(th); sin(th)];
            du = J \ dvec;
            assignin('base', 'dux', 0);  assignin('base', 'duy', 0);
            assignin('base', 'dux_step', du(1));  assignin('base', 'duy_step', du(2));

            out = sim(mdl);
            [t, Xs] = get_log(out, 'log_X');
            [~, Ys] = get_log(out, 'log_Y');
            [ST(iv,is), OS(iv,is), RD(iv,is)] = metric(t - step_time, [Xs Ys], dvec, band, to_zero);
        end
        fprintf('  %s = %g 완료\n', which, vals(iv));
    end
    T = table(vals(:), mean(ST, 2, 'omitnan'), max(OS, [], 2), mean(RD, 2), sum(isnan(ST), 2), ...
        'VariableNames', {which, 'settle_ms', 'os_pct', 'reduce_pct', 'fail'});
end

function [t, v] = get_log(out, name)
    s = out.get(name);
    if isa(s, 'timeseries'),  t = s.Time;  v = squeeze(s.Data);
    elseif isstruct(s),       t = s.time;  v = squeeze(s.signals.values);
    else,                     t = out.tout; v = s;
    end
    t = t(:);  v = v(:);
end

function [st, os, rd] = metric(t, P, dvec, band, to_zero)
% st: 기준점 주변 2 % 안에 자리 잡는 시간 [ms]
% os: 기준점을 지나 반대쪽으로 넘어간 정도 [% of 외란]
% rd: 오차 감소율 [%] = (1 - 최종 오차/외란) x 100
    m = t >= 0;  t = t(m);  P = P(m, :);
    d = norm(dvec);  dh = dvec / d;
    proj  = P * dh;                                   % 외란 방향 성분
    final = mean(proj(t >= t(end) - 0.2));            % 최종값 (마지막 0.2 s 평균)
    if to_zero, ref = 0; else, ref = final; end
    e = sqrt(sum((P - ref*dh.').^2, 2));              % 기준점까지 거리
    [~, ipk] = max(proj);
    os = max(0, ref - min(proj(ipk:end))) / d * 100;
    rd = (1 - final/d) * 100;
    last_out = find(e > band*d, 1, 'last');
    if isempty(last_out),       st = 0;
    elseif last_out < numel(t), st = 1e3 * t(last_out + 1);
    else,                       st = NaN;
    end
end