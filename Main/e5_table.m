function S = e5_table(RES, fld, est_names)
%E5_TABLE  추정기 × 외란 구간 × 그룹(NaN없음 / NaN있음 / 전체) 비교표
%   S = e5_table(RES, fld, est_names)
%   RES  : 구조체 배열 (est_id, 그리고 fld 필드에 이벤트 표). 같은 est_id가 여러 개면 합침 (seed 합산)
%   fld  : 'EV' (실제 위치 기준) 또는 'EVe' (정규화 X̂ 기준)
%   평균·중앙값: coarse 이벤트를 뺀 값 / 횟수 열(n, nan_n, loss, coarse_*, sat, search): 그룹 전체
%   TEST_closed_loop, E5_summary 공용

bins  = [0 1; 1 2; 2 3; 3 Inf];
blbl  = {'0~1 mm', '1~2 mm', '2~3 mm', '3 mm~'};
glbl  = {'NaN없음', 'NaN있음', '전체'};
S = table();
ids = unique([RES.est_id], 'stable');
for id = ids
    EV = table();
    for k = find([RES.est_id] == id)
        if ~isempty(RES(k).(fld)) && height(RES(k).(fld)) > 0
            EV = [EV; RES(k).(fld)]; %#ok<AGROW>
        end
    end
    if height(EV) == 0, continue; end
    for b = 1:size(bins, 1)
        mb = EV.dist_mm >= bins(b,1) & EV.dist_mm < bins(b,2);
        for g = 1:3
            switch g
                case 1, m = mb & EV.nan_n == 0;
                case 2, m = mb & EV.nan_n > 0;
                case 3, m = mb;
            end
            if ~any(m), continue; end
            ok = m & EV.coarse == 0;            % 평균에 쓰는 이벤트
            S = [S; table(string(est_names{id}), string(blbl{b}), string(glbl{g}), sum(m), sum(ok), ...
                mean(EV.peak_mm(ok) ./ EV.dist_mm(ok), 'omitnan'), ...
                median(EV.settle_ms(ok), 'omitnan'), mx(EV.settle_ms(ok)), ...
                median(EV.os_pct(ok), 'omitnan'), mean(EV.rmse_um(ok), 'omitnan'), ...
                median(EV.ret_ms(ok), 'omitnan'), sum(isnan(EV.settle_ms(ok))), ...
                sum(EV.nan_n(m)), sum(EV.loss(m)), sum(EV.coarse(m) == 1), sum(EV.coarse(m) == 2), ...
                sum(EV.sat(m)), sum(EV.search(m)), ...
                'VariableNames', {'estimator', 'dist', 'group', 'n', 'n_avg', 'peak_ratio', ...
                'settle_med_ms', 'settle_max_ms', 'os_med_pct', 'rmse_um', 'ret_med_ms', 'no_settle', ...
                'nan_n', 'loss', 'coarse_loss', 'coarse_nan', 'sat', 'search'})]; %#ok<AGROW>
        end
    end
end
end

function v = mx(x)
x = x(isfinite(x));
if isempty(x), v = NaN; else, v = max(x); end
end
