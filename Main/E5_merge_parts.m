%% E5_merge_parts.m
% 중간에 꺼진 실행의 추정기별 임시 파일(E5_PART_*)을 seed 하나로 묶어 E5_CMP 파일로 저장
%   추정기마다 가장 최근 PART 파일을 씀 (version, alpha가 같은 것만)
%   끝까지 돈 실행은 이미 E5_CMP가 저장되므로 돌리지 말 것 (중복)
seed       = 3;                 % 묶을 seed
ver        = 4;                 % 판정 버전
alpha_list = [1 0 0.9 0.8 0.5 0.3 0.1];     % NEW_TEST와 같게

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

%% 추정기별 최신 PART 고르기
f = dir(fullfile(save_dir, sprintf('E5_PART_seed%d_est*.mat', seed)));
[~, o] = sort([f.datenum]);  f = f(o);
pick = cell(1, numel(alpha_list));
for i = 1:numel(f)
    D = load(fullfile(f(i).folder, f(i).name), 'cfg', 'R', 'alpha_list');
    id = D.R.est_id;
    if D.cfg.version ~= ver || numel(D.alpha_list) < id || D.alpha_list(id) ~= alpha_list(id), continue; end
    pick{id} = f(i);            % 시간순이라 나중 파일이 덮어씀 = 최신
end
ids = find(~cellfun(@isempty, pick));
if isempty(ids), error('seed %d의 PART 파일이 없음', seed); end
[~, o] = sort(cellfun(@(x) x.datenum, pick(ids)));  ids = ids(o);   % 실제로 돈 순서

%% 합치기
for n = 1:numel(ids)
    D = load(fullfile(pick{ids(n)}.folder, pick{ids(n)}.name));
    fprintf('사용: %s\n', pick{ids(n)}.name);
    if n == 1
        RES = D.R;  K = D;
    else
        if ~isequal(D.ev_t, K.ev_t), warning('외란 일정이 다름: %s', pick{ids(n)}.name); end
        RES(end+1) = D.R;                                                  %#ok<SAGROW>
    end
end

cfg = K.cfg;  cfg.est_list = ids;  cfg.merged = true;   % est_list = 실제로 돈 순서
off = K.off;  ev_t = K.ev_t;  ev_d = K.ev_d;  est_names = K.est_names;
TX = K.TX;  TY = K.TY;  lossX = K.lossX;  lossY = K.lossY;  track_mm = K.track_mm;  vib_mm = K.vib_mm;
out = fullfile(save_dir, sprintf('E5_CMP_seed%d_%s.mat', seed, datestr(now, 'yyyymmdd_HHMMSS')));
save(out, 'RES', 'cfg', 'off', 'ev_t', 'ev_d', 'est_names', ...
    'TX', 'TY', 'lossX', 'lossY', 'track_mm', 'vib_mm');
fprintf('저장: %s (순서 %s)\n', out, mat2str(ids));