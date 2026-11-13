%% E5_summary.m
% 같은 판정 버전(cfg.version)의 E5_CMP_seed*.mat을 모두 합쳐 비교표, NaN 상황표, 그림을 만든다.
%   기준이 다른 옛 파일(version 없음)은 자동으로 빠짐
%   order: 그 실행에서 추정기가 몇 번째로 돌았는지 (순서 반전 확인용)
%   v3: 이상적 센서(추정값 = 실제 위치 + 측정 노이즈)를 같은 외란으로 시뮬레이션해 기준(= 1)으로 삼음   % [수정]
%     표 A: seed별 전체 RMSE, 이상적 센서 대비 비율                                                   % [수정]
%     표 B: 외란 크기 구간별 이벤트 구간 RMSE(외란 순간 ~ 다음 외란 전), 비율, 오버슈트, 정착 시간    % [수정]
%     탐색 구간(실제 위치 모름) 샘플은 하드웨어·시뮬레이션 모두 RMSE에서 뺌                           % [수정]
init_pid_sim;                   % Ged (E4 이산 모델), Ts                                              % [수정]
ver = 3;                        % 합칠 판정 버전                                                      % [수정]
bin_w = 0.5;                    % [mm] 표 B 구간 폭                                                   % [수정]
est_names = {'DoS', 'LogRatio', 'Hybrid a=0.9', 'Hybrid a=0.8'};
%% 경로
here = fileparts(mfilename('fullpath'));  if isempty(here), here = pwd; end
root = here;
while ~isfolder(fullfile(root, 'Main'))
    parent = fileparts(root);
    if strcmp(parent, root), error('프로젝트 루트(Main 폴더)를 찾지 못함'); end
    root = parent;
end
addpath(genpath(root));
save_dir = fullfile(root, 'Main', 'DATA');
[nzx, nzy] = load_noise(save_dir);  sn = [std(nzx) std(nzy)];                                       % [수정]
fprintf('이상적 센서 시뮬레이션 노이즈: X %.2f / Y %.2f µm\n', 1e3*sn);                                % [수정]
%% 파일 모으기
f = dir(fullfile(save_dir, 'E5_CMP_seed*.mat'));
[~, o] = sort([f.datenum]);  f = f(o);
ALL = struct('est_id', {}, 'EV', {}, 'EVe', {}, 'NV', {});
used = {};  track_mm = NaN;  loss_mm = NaN;
TA = table();  W = table();                                                                          % [수정]
for i = 1:numel(f)
    D = load(fullfile(f(i).folder, f(i).name), 'RES', 'cfg', 'track_mm', 'lossX', 'lossY', ...
             'ev_t', 'ev_d', 'vib_mm', 'TX', 'TY');                                                  % [수정]
    if ~isfield(D, 'cfg') || ~isfield(D.cfg, 'version') || D.cfg.version ~= ver, continue; end
    if ~isfield(D, 'RES') || isempty(D.RES), continue; end
    used{end+1} = sprintf('%s (seed %d, 순서 %s)', f(i).name, D.cfg.seed, mat2str(D.cfg.est_list)); %#ok<SAGROW>
    [ide_tot, ide_ev] = ideal_run(D, Ged, Ts, sn);                                                   % [수정]
    for k = 1:numel(D.RES)
        r = D.RES(k);
        od = find(D.cfg.est_list == r.est_id, 1);
        EV = r.EV;   EV.seed  = repmat(D.cfg.seed, height(EV), 1);   EV.order  = repmat(od, height(EV), 1);
        EVe = r.EVe; EVe.seed = repmat(D.cfg.seed, height(EVe), 1); EVe.order = repmat(od, height(EVe), 1);
        NV = r.NV;   NV.seed  = repmat(D.cfg.seed, height(NV), 1);   NV.order  = repmat(od, height(NV), 1);
        ALL(end+1) = struct('est_id', r.est_id, 'EV', EV, 'EVe', EVe, 'NV', NV); %#ok<SAGROW>
        wr = event_rms(r.L, D.ev_t, D.cfg.T_total);                                                  % [수정]
        ne = height(r.EV);  ev = r.EV.event;                                                         % [수정]
        W = [W; table(repmat(D.cfg.seed, ne, 1), repmat(r.est_id, ne, 1), repmat(od, ne, 1), ev, ...
             r.EV.dist_mm, wr(ev), ide_ev(ev), r.EV.os_pct, r.EV.settle_ms, r.EV.coarse, r.EV.nan_n, ...
             'VariableNames', {'seed', 'est_id', 'order', 'event', 'dist_mm', 'win_um', 'ideal_um', ...
             'os_pct', 'settle_ms', 'coarse', 'nan_n'})]; %#ok<AGROW>                               % [수정]
        TA = [TA; table(D.cfg.seed, string(est_names{r.est_id}), r.tot.rms_um, ide_tot, ...
              r.tot.rms_um/ide_tot, 'VariableNames', {'seed', 'estimator', 'rms_um', 'ideal_um', 'ratio'})]; %#ok<AGROW>  % [수정]
    end
    track_mm = D.track_mm;  loss_mm = min(abs([D.lossX D.lossY]));
    clear D                                                                                          % [수정]
end
if isempty(ALL), error('version %d 파일이 없음. 새 TEST_closed_loop으로 실행한 결과가 필요함', ver); end
fprintf('합친 파일 %d개:\n', numel(used));  fprintf('  %s\n', used{:});
%% 표
S  = e5_table(ALL, 'EV',  est_names);
Se = e5_table(ALL, 'EVe', est_names);
fprintf('\n그룹: NaN없음 / NaN있음 / 전체. 평균·중앙값은 coarse 이벤트를 뺀 값, 횟수 열은 전체\n');
fprintf('\n=== [실제 위치 기준] 합계 ===\n');  disp(S)
fprintf('\n=== [정규화 X̂ 기준] 합계 ===\n');  disp(Se)
NVall = table();
for k = 1:numel(ALL)
    if height(ALL(k).NV) == 0, continue; end
    T = ALL(k).NV;  T.estimator = repmat(string(est_names{ALL(k).est_id}), height(T), 1);
    NVall = [NVall; movevars(T, 'estimator', 'Before', 1)]; %#ok<AGROW>
end
fprintf('\n=== NaN 상황 (NaN이 난 이벤트마다 한 줄) ===\n');
if height(NVall) > 0, disp(NVall), else, disp('NaN 없음'); end
%% 표 A: 전체 RMSE, 이상적 센서 대비                                                                 % [수정]
fprintf('\n=== 표 A: seed별 전체 RMSE [µm], ratio = 추정기 / 이상적 센서 (1 = 이상적) ===\n');        % [수정]
disp(TA)                                                                                             % [수정]
GA = groupsummary(TA, 'estimator', {'mean', 'std'}, {'rms_um', 'ratio'});                            % [수정]
fprintf('=== 표 A 요약 (seed %d개) ===\n', numel(unique(TA.seed)));  disp(GA)                        % [수정]
%% 표 B: 외란 크기 구간별 (coarse 이벤트 제외)                                                       % [수정]
edges = 0:bin_w:4;  TB = table();                                                                    % [수정]
for b = 1:numel(edges)-1                                                                             % [수정]
    for id = 1:4                                                                                     % [수정]
        m = W.est_id == id & W.dist_mm >= edges(b) & W.dist_mm < edges(b+1) & W.coarse == 0;          % [수정]
        if ~any(m), continue; end                                                                    % [수정]
        TB = [TB; table(sprintf("%.1f~%.1f", edges(b), edges(b+1)), string(est_names{id}), nnz(m), ...
             median(W.win_um(m)./W.ideal_um(m), 'omitnan'), median(W.win_um(m), 'omitnan'), ...
             median(W.ideal_um(m), 'omitnan'), median(W.os_pct(m), 'omitnan'), ...
             median(W.settle_ms(m), 'omitnan'), nnz(m & W.nan_n > 0), ...
             'VariableNames', {'dist_mm', 'estimator', 'n', 'ratio_med', 'win_rms_um', 'ideal_rms_um', ...
             'os_med_pct', 'settle_med_ms', 'nan_events'})]; %#ok<AGROW>                            % [수정]
    end                                                                                              % [수정]
end                                                                                                  % [수정]
fprintf('\n=== 표 B: 외란 크기별 (이벤트 구간 = 외란 순간 ~ 다음 외란 전, 진동 포함) ===\n');           % [수정]
fprintf('ratio_med: 같은 이벤트의 추정기 RMSE / 이상적 센서 RMSE 중앙값 (1 = 이상적)\n');              % [수정]
fprintf('os_med_pct: 1 mm 미만은 진동이 외란에 비해 커서 오버슈트가 부풀려짐\n');                    % [수정]
disp(TB)                                                                                             % [수정]
%% 순서 영향 확인: 추정기별 실행 순서에 따른 정착 시간(0~2 mm), RMSE 비율 중앙값 (coarse 제외)         % [수정]
fprintf('\n=== 실행 순서별 정착 시간 [ms] (0~2 mm) / RMSE 비율 (전체 크기), coarse 제외 ===\n');      % [수정]
for id = 1:4                                                                                         % [수정]
    m = W.est_id == id & W.coarse == 0;                                                              % [수정]
    fprintf('%-14s', est_names{id});                                                                 % [수정]
    for q = unique(W.order(m))'                                                                      % [수정]
        mq = m & W.order == q;                                                                       % [수정]
        fprintf('  %d번째: %.0f ms, %.3f (n=%d)', q, median(W.settle_ms(mq & W.dist_mm < 2), 'omitnan'), ...
            median(W.win_um(mq)./W.ideal_um(mq), 'omitnan'), nnz(mq));                               % [수정]
    end                                                                                              % [수정]
    fprintf('\n');                                                                                   % [수정]
end                                                                                                  % [수정]
%% 그림
seeds = unique(cellfun(@(s) sscanf(regexp(s, 'seed \d+', 'match', 'once'), 'seed %d'), used));
e5_plot(ALL, est_names, track_mm, loss_mm, sprintf('seed %s 합계', mat2str(seeds)));
C = lines(4);  mk = {'o', 's', '^', 'd'};                                                            % [수정]
figure('Name', 'E5 v3 이상적 센서 대비', 'Color', 'w', 'Position', [100 100 1200 450]);              % [수정]
subplot(1, 2, 1);  hold on;  grid on;                                                                % [수정]
mu = NaN(1, 4);  sd = mu;                                                                            % [수정]
for id = 1:4, rr = TA.ratio(TA.estimator == est_names{id});  mu(id) = mean(rr);  sd(id) = std(rr); end  % [수정]
bb = bar(1:4, mu, 'FaceColor', 'flat');  bb.CData = C;                                               % [수정]
errorbar(1:4, mu, sd, 'k.', 'LineWidth', 1.2);                                                       % [수정]
yline(1, 'k--', '이상적 센서');                                                                      % [수정]
text(1:4, mu + sd, compose('%.3f', mu), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');  % [수정]
set(gca, 'XTick', 1:4, 'XTickLabel', est_names);  ylim([0.9, max(mu + sd) + 0.05]);                  % [수정]
ylabel('전체 RMSE / 이상적 센서');                                                                   % [수정]
title(sprintf('전체 RMSE (seed %d개 평균 ± 표준편차)', numel(unique(TA.seed))));                      % [수정]
subplot(1, 2, 2);  hold on;  grid on;                                                                % [수정]
ctr = edges(1:end-1) + bin_w/2;                                                                      % [수정]
for id = 1:4                                                                                         % [수정]
    md = NaN(size(ctr));                                                                             % [수정]
    for b = 1:numel(ctr)                                                                             % [수정]
        m = W.est_id == id & W.dist_mm >= edges(b) & W.dist_mm < edges(b+1) & W.coarse == 0;          % [수정]
        md(b) = median(W.win_um(m)./W.ideal_um(m), 'omitnan');                                       % [수정]
    end                                                                                              % [수정]
    plot(ctr, md, ['-' mk{id}], 'Color', C(id,:), 'MarkerFaceColor', C(id,:), 'LineWidth', 1.5, ...
        'DisplayName', est_names{id});                                                               % [수정]
end                                                                                                  % [수정]
yline(1, 'k--', 'HandleVisibility', 'off');                                                          % [수정]
xlabel('외란 크기 [mm]');  ylabel('이벤트 구간 RMSE / 이상적 센서 (중앙값)');  legend('Location', 'northwest');  % [수정]
title('외란 크기별 (외란 순간 ~ 다음 외란 전)');                                                     % [수정]
out = fullfile(save_dir, sprintf('E5_SUMMARY_v%d_%s.mat', ver, datestr(now, 'yyyymmdd_HHMMSS')));
save(out, 'S', 'Se', 'NVall', 'ALL', 'used', 'track_mm', 'loss_mm', 'TA', 'GA', 'TB', 'W');          % [수정]
fprintf('저장: %s\n', out);

%% local functions                                                                                   % [수정]
function wr = event_rms(L, ev_t, T_end)                                                              % [수정]
% 이벤트별 구간 RMSE [µm] (외란 순간 ~ 다음 외란 전, 실제 위치, 탐색 구간 NaN 제외)                  % [수정]
    w = L.new;  t = L.t(w);  e = 1e3*sqrt(sum(L.true(w,:).^2, 2));                                    % [수정]
    ne = numel(ev_t);  wr = NaN(ne, 1);                                                              % [수정]
    for j = 1:ne                                                                                     % [수정]
        t2 = T_end;  if j < ne, t2 = ev_t(j+1); end                                                  % [수정]
        m = t >= ev_t(j) & t < t2;                                                                   % [수정]
        if nnz(m) > 5, wr(j) = sqrt(mean(e(m).^2, 'omitnan')); end                                   % [수정]
    end                                                                                              % [수정]
end                                                                                                  % [수정]

function [tot, ev] = ideal_run(D, G, Ts, sn)                                                         % [수정]
% 이상적 센서(추정값 = 실제 위치 + 측정 노이즈)로 같은 외란(계단 + SILEX)을 시뮬레이션               % [수정]
% tot: 첫 외란부터 끝까지 RMSE [µm], ev: 이벤트별 구간 RMSE [µm] (추적 구간 밖 샘플 제외)             % [수정]
    c = D.cfg;  N = floor(c.T_total/Ts);  t = (0:N-1).'*Ts;                                          % [수정]
    cum = [zeros(2, 1) cumsum(D.ev_d, 2)];                                                           % [수정]
    np = sum(t >= D.ev_t(:).', 2);                         % 지난 외란 수                            % [수정]
    iv = min(floor(t/c.Ts_vib) + 1, size(D.vib_mm, 1));                                              % [수정]
    d  = cum(:, np + 1).' + D.vib_mm(iv, :);               % 미러 명령에 더해지는 외란 [mm]          % [수정]
    C  = pid(c.Kp, c.Ki, 0, 0, Ts, 'IFormula', 'BackwardEuler');                                     % [수정]
    Sd = feedback(G, C);  Tn = feedback(C*G, 1);           % 외란 → 위치, 노이즈 → 위치(부호 -)      % [수정]
    rs = RandStream('mt19937ar', 'Seed', 2000 + c.seed);                                             % [수정]
    Y = zeros(N, 2);                                                                                 % [수정]
    for k = 1:2                                                                                      % [수정]
        Y(:,k) = lsim(Sd, d(:,k), t) - lsim(Tn, sn(k)*randn(rs, N, 1), t);                           % [수정]
    end                                                                                              % [수정]
    in = Y(:,1) >= D.TX.P(1) & Y(:,1) <= D.TX.P(end) & Y(:,2) >= D.TY.P(1) & Y(:,2) <= D.TY.P(end);  % [수정]
    e = 1e3*sqrt(sum(Y.^2, 2));  e(~in) = NaN;                                                       % [수정]
    tot = sqrt(mean(e(t >= D.ev_t(1)).^2, 'omitnan'));                                               % [수정]
    ne = numel(D.ev_t);  ev = NaN(ne, 1);                                                            % [수정]
    for j = 1:ne                                                                                     % [수정]
        t2 = c.T_total;  if j < ne, t2 = D.ev_t(j+1); end                                            % [수정]
        m = t >= D.ev_t(j) & t < t2;                                                                 % [수정]
        if nnz(m) > 5, ev(j) = sqrt(mean(e(m).^2, 'omitnan')); end                                   % [수정]
    end                                                                                              % [수정]
end                                                                                                  % [수정]

function [nx, ny] = load_noise(data_dir)                                                             % [수정]
% 최신 NOISE_*.mat를 DoS 중심 기울기로 위치 잡음 [mm]로 바꿈 (tune_fine과 같음)                       % [수정]
    f = dir(fullfile(data_dir, 'NOISE_*.mat'));  [~, i] = max([f.datenum]);                          % [수정]
    R = load(fullfile(f(i).folder, f(i).name));                                                      % [수정]
    s.XDiff = R.Raw(:,1); s.YDiff = R.Raw(:,2); s.SUM = R.Raw(:,3);                                  % [수정]
    [~, ~, dx, dy] = estimator_dos(s, 1);                                                            % [수정]
    sl = [0 0];  ax = 'XY';                                                                          % [수정]
    for k = 1:2                                                                                      % [수정]
        g = dir(fullfile(data_dir, ['BEAM_SCAN_' ax(k) '_*.mat']));  [~, j] = max([g.datenum]);       % [수정]
        B = load(fullfile(g(j).folder, g(j).name));  mk = abs(B.X(:)) <= 0.3;                        % [수정]
        b.XDiff = B.M(mk,1); b.YDiff = B.M(mk,2); b.SUM = B.M(mk,3);                                 % [수정]
        [~, ~, ex, ey] = estimator_dos(b, 1);                                                        % [수정]
        if k == 1, p = polyfit(B.X(mk), ex, 1); else, p = polyfit(B.X(mk), ey, 1); end                % [수정]
        sl(k) = p(1);                                                                                % [수정]
    end                                                                                              % [수정]
    nx = (dx - mean(dx)) / sl(1);  ny = (dy - mean(dy)) / sl(2);                                     % [수정]
end                                                                                                  % [수정]