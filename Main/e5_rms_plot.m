function e5_rms_plot(RES, est_names, ev_t, ev_d, ttl)
% 전체 RMS와 이벤트별 구간 RMS (외란 순간 ~ 다음 외란 전, 실제 위치, 탐색 구간 NaN 제외)
mk = {'o', 's', '^', 'd', 'v', '>', 'p'};  C = lines(7);          % [수정] est_id별 색·모양 (e5_plot과 같음)
ne = numel(RES);  names = strings(ne, 1);  V = zeros(ne, 2);
for i = 1:ne
    names(i) = est_names{RES(i).est_id};
    V(i,:) = [RES(i).tot.rms_um, RES(i).noise_um];
end
figure('Name', ['E5 RMS (' ttl ')'], 'Color', 'w', 'Position', [100 100 1200 450]);

subplot(1, 2, 1);
b = bar(categorical(names, names), V);  grid on;  ylabel('RMS [µm]');
for k = 1:2
    text(b(k).XEndPoints, b(k).YEndPoints, compose('%.1f', b(k).YData), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
legend(b, {'전체 (계단 + 진동)', '정상 상태 (진동만)'}, 'Location', 'northwest');
title('전체 RMS');

subplot(1, 2, 2);  hold on;  grid on;
edges = 0:0.5:4;  ctr = edges(1:end-1) + 0.25;  d = vecnorm(ev_d).';  nev = numel(ev_t);
for i = 1:ne
    L = RES(i).L;  w = L.new;  t = L.t(w);  e = 1e3*sqrt(sum(L.true(w,:).^2, 2));
    r = NaN(nev, 1);
    for j = 1:nev
        t2 = t(end);  if j < nev, t2 = ev_t(j+1); end
        m = t >= ev_t(j) & t < t2;
        if nnz(m) > 5, r(j) = sqrt(mean(e(m).^2, 'omitnan')); end
    end
    id = RES(i).est_id;
    scatter(d, r, 15, C(id,:), 'filled', 'MarkerFaceAlpha', 0.3, 'HandleVisibility', 'off');
    md = arrayfun(@(k) median(r(d >= edges(k) & d < edges(k+1)), 'omitnan'), 1:numel(ctr));
    plot(ctr, md, ['-' mk{id}], 'Color', C(id,:), 'MarkerFaceColor', C(id,:), ...
        'LineWidth', 1.5, 'DisplayName', names(i));
end
xlabel('외란 크기 [mm]');  ylabel('RMS [µm]');  legend('Location', 'northwest');
title('이벤트별 RMS (외란 순간 ~ 다음 외란 전, 진동 포함)');
sgtitle([ttl ' / 실제 위치 기준, 탐색 구간(NaN) 샘플 제외']);
end