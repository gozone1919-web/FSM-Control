function e5_plot(RES, est_names, track_mm, loss_mm, ttl)
%E5_PLOT  추정기 비교 그림 (실제 위치 기준)
%   e5_plot(RES, est_names, track_mm, loss_mm, ttl)
%   그림 1 (2×2): 정착 시간, 오버슈트, 최대 오차/외란, 추적 구간 복귀 시간
%   그림 2 (1×2): 이벤트별 NaN 시간, 구간별 NaN 발생 비율
%   선: 0.5 mm 구간 중앙값 (coarse 이벤트 제외)
%   점: 채움 = NaN 없음, 빈 점 = NaN 있음, x = coarse (그림 1 정착 시간 칸, y = 0)
%   파란 점선: 추적 구간 끝, 빨간 점선: 광량 손실 시작
%   RES는 est_id, EV 필드를 가진 구조체 배열 (같은 est_id 여러 개면 합침)
%   TEST_closed_loop, E5_summary 공용

cols  = lines(7);                                   % [수정] 추정기 7개
mk    = {'o', 's', '^', 'd', 'v', '>', 'p'};       % [수정]
edges = 0:0.5:4;  ctr = edges(1:end-1) + 0.25;
ids   = unique([RES.est_id], 'stable');
EVs   = cell(1, numel(ids));
for i = 1:numel(ids)
    EV = table();
    for k = find([RES.est_id] == ids(i))
        if ~isempty(RES(k).EV) && height(RES(k).EV) > 0, EV = [EV; RES(k).EV]; end %#ok<AGROW>
    end
    EVs{i} = EV;
end

%% 그림 1: 성능 지표
fld = {'settle_ms', 'os_pct', 'peak', 'ret_ms'};
tl  = {'정착 시간 [ms]', '오버슈트 [%]', '최대 오차 / 외란 (탐색 구간 미진입만)', '추적 구간 복귀 시간 [ms]'};
figure('Name', sprintf('E5 비교 (%s)', ttl), 'Color', 'w', 'Position', [40 40 1300 820]);
for p = 1:4
    ax = subplot(2, 2, p); hold(ax, 'on'); grid(ax, 'on');
    for i = 1:numel(ids)
        EV = EVs{i};  if height(EV) == 0, continue; end
        c  = cols(ids(i), :);
        ok = EV.coarse == 0;
        if strcmp(fld{p}, 'peak'), y = EV.peak_mm ./ EV.dist_mm; else, y = EV.(fld{p}); end
        a = ok & EV.nan_n == 0;  b = ok & EV.nan_n > 0;
        scatter(ax, EV.dist_mm(a), y(a), 22, c, 'filled', 'Marker', mk{ids(i)}, ...
            'MarkerFaceAlpha', 0.25, 'MarkerEdgeAlpha', 0.25, 'HandleVisibility', 'off');
        scatter(ax, EV.dist_mm(b), y(b), 30, c, 'Marker', mk{ids(i)}, ...
            'MarkerEdgeAlpha', 0.6, 'LineWidth', 1, 'HandleVisibility', 'off');
        md = bin_median(EV.dist_mm(ok), y(ok), edges);
        v  = ~isnan(md);
        plot(ax, ctr(v), md(v), '-', 'Color', c, 'Marker', mk{ids(i)}, 'MarkerFaceColor', c, ...
            'LineWidth', 1.8, 'MarkerSize', 6, 'DisplayName', est_names{ids(i)});
        if p == 1 && any(~ok)
            scatter(ax, EV.dist_mm(~ok), zeros(sum(~ok), 1), 70, c, 'x', 'LineWidth', 1.8, ...
                'HandleVisibility', 'off');
        end
    end
    xline(ax, track_mm, 'b--', 'HandleVisibility', 'off');
    if ~isnan(loss_mm), xline(ax, loss_mm, 'r--', 'HandleVisibility', 'off'); end
    xlim(ax, [0 4]);  xlabel(ax, '외란 크기 [mm]');  title(ax, tl{p});
    if p == 1, legend(ax, 'Location', 'northwest'); end
end
sgtitle(sprintf(['%s / 실제 위치 기준  (선: 0.5 mm 구간 중앙값, 채운 점: NaN 없음, 빈 점: NaN 있음, ' ...
    'x: coarse, 파란 점선: 추적 구간 끝, 빨간 점선: 광량 손실 시작)'], ttl), 'FontSize', 10);

%% 그림 2: NaN (추정 불가)
figure('Name', sprintf('E5 NaN (%s)', ttl), 'Color', 'w', 'Position', [80 80 1300 420]);
ax1 = subplot(1, 2, 1); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = subplot(1, 2, 2); hold(ax2, 'on'); grid(ax2, 'on');
for i = 1:numel(ids)
    EV = EVs{i};  if height(EV) == 0, continue; end
    c  = cols(ids(i), :);
    scatter(ax1, EV.dist_mm, EV.nan_ms, 22, c, 'filled', 'Marker', mk{ids(i)}, ...
        'MarkerFaceAlpha', 0.3, 'HandleVisibility', 'off');
    md = bin_median(EV.dist_mm, EV.nan_ms, edges);  v = ~isnan(md);
    plot(ax1, ctr(v), md(v), '-', 'Color', c, 'Marker', mk{ids(i)}, 'MarkerFaceColor', c, ...
        'LineWidth', 1.8, 'DisplayName', est_names{ids(i)});
    fr = bin_mean(EV.dist_mm, double(EV.nan_n > 0), edges);       v = ~isnan(fr);
    fc = bin_mean(EV.dist_mm, double(EV.coarse == 2), edges);
    plot(ax2, ctr(v), 100*fr(v), '-', 'Color', c, 'Marker', mk{ids(i)}, 'MarkerFaceColor', c, ...
        'LineWidth', 1.8, 'DisplayName', [est_names{ids(i)} ' NaN 발생']);
    plot(ax2, ctr(v), 100*fc(v), ':', 'Color', c, 'LineWidth', 1.5, ...
        'DisplayName', [est_names{ids(i)} ' NaN coarse']);
end
for ax = [ax1 ax2]
    xline(ax, track_mm, 'b--', 'HandleVisibility', 'off');
    if ~isnan(loss_mm), xline(ax, loss_mm, 'r--', 'HandleVisibility', 'off'); end
    xlim(ax, [0 4]);  xlabel(ax, '외란 크기 [mm]');
end
title(ax1, '이벤트별 NaN 시간 [ms] (선: 구간 중앙값)');  legend(ax1, 'Location', 'northwest');
title(ax2, '구간별 NaN 발생 비율 [%] (실선: NaN 발생, 점선: NaN으로 coarse)');
ylim(ax2, [0 105]);  legend(ax2, 'Location', 'northwest', 'NumColumns', 2);
sgtitle(sprintf('%s / 추정 불가(NaN): LogRatio·Hybrid의 바닥값 구간 영향', ttl), 'FontSize', 10);
end

function md = bin_median(x, y, edges)
md = NaN(1, numel(edges) - 1);
for b = 1:numel(edges) - 1
    m = x >= edges(b) & x < edges(b+1) & isfinite(y);
    if any(m), md(b) = median(y(m)); end
end
end

function mn = bin_mean(x, y, edges)
mn = NaN(1, numel(edges) - 1);
for b = 1:numel(edges) - 1
    m = x >= edges(b) & x < edges(b+1);
    if any(m), mn(b) = mean(y(m)); end
end
end
