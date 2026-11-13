%% estimator_compare_exp.m
% 실측 데이터로 세 추정기 비교 (시뮬레이션 figure 1의 실측 버전)
%   빔 스캔 데이터(±3.3 mm)의 XDiff, SUM -> DoS / LogRatio / Hybrid δ
%   E3에서 잰 k로 X_hat = k·δ,  실제 X = J_x·u
% 필요: Main\DATA 안의 빔 스캔 파일과 E3_CAL 파일

here         = fileparts(mfilename('fullpath'));   % QPD\Experiment
qpd_dir      = fileparts(here);                    % QPD
project_root = fileparts(qpd_dir);                 % 프로젝트 루트
data_dir     = fullfile(project_root, 'Main', 'DATA');
addpath(qpd_dir);                                  % estimator_*.m 함수 경로

alpha    = 0.9;
tol_rel  = 0.05;                                     % 선형 범위 기준 (상대오차 5%)

% 가장 최근 파일 불러오기
fb = dir(fullfile(data_dir, '*beam_scan*.mat'));
fc = dir(fullfile(data_dir, 'E3_CAL_*.mat'));
assert(~isempty(fb) && ~isempty(fc), 'DATA 폴더에 빔 스캔 또는 E3_CAL 파일이 없음');
[~, ib] = max([fb.datenum]);  B = load(fullfile(data_dir, fb(ib).name));
[~, ic] = max([fc.datenum]);  C = load(fullfile(data_dir, fc(ic).name));
fprintf('빔 스캔: %s\nE3 보정: %s\n', fb(ib).name, fc(ic).name);

% 세 추정기 δ (dark 제거된 점별 평균값 사용)
sig.XDiff = B.M(:,1).';  sig.YDiff = B.M(:,2).';  sig.SUM = B.M(:,3).';
[~, ~, d1] = estimator_dos(sig, 1);
[~, ~, d2] = estimator_logratio(sig, 1);
if nargin('estimator_hybrid') == 4
    d3 = estimator_hybrid(sig, alpha);
else
    d3 = estimator_hybrid(sig, 1, alpha);
end
D = [d1(:), d2(:), d3(:)];

% E3 k (X축)로 환산
k  = C.cal.k_model_mm(:,1).';                        % [DoS LogRatio Hybrid] [mm]
X  = B.X(:);                                         % 실제 위치 [mm] = J_x·u
Xh = D .* k;                                         % 추정 위치 [mm]
names = {'DoS', 'LogRatio', 'Hybrid'};

% 선형 범위: 중심에서 바깥으로 |X_hat - X| <= tol_rel·|X| 가 유지되는 최대 |X| (양쪽 중 작은 값)
lin = zeros(1,3);
[~, i0] = min(abs(X));
for m = 1:3
    ok = abs(Xh(:,m) - X) <= tol_rel*abs(X) | abs(X) < 1e-9;
    r = [0 0];
    for s = [1 -1]
        idx = i0;
        while idx+s >= 1 && idx+s <= numel(X) && ok(idx+s), idx = idx + s; end
        r((3-s)/2) = abs(X(idx));
    end
    lin(m) = min(r);
end

fprintf('\n=== 실측 추정기 비교 ===\n');
fprintf('%10s %10s %18s\n', '', 'k [mm]', '선형 범위 [mm]');
for m = 1:3
    fprintf('%10s %10.4f %18.3f\n', names{m}, k(m), lin(m));
end

% 그림
figure('Name', 'Estimator comparison (measured)', 'Position', [100 100 1100 450]);
subplot(1,2,1); hold on; grid on;
plot(X, Xh, 'o-', 'MarkerSize', 4);
plot(X, X, 'k--');
xlabel('X true [mm] (J_x \times u)'); ylabel('X\_hat [mm]');
title('실측: 추정 위치 vs 실제 위치');
legend([names, {'y = x'}], 'Location', 'best');
subplot(1,2,2); hold on; grid on;
plot(X, Xh - X, 'o-', 'MarkerSize', 4);
yline(0, 'k--', 'HandleVisibility', 'off');
xlabel('X true [mm]'); ylabel('X\_hat - X [mm]');
title('실측: 추정 오차');
legend(names, 'Location', 'best');