function info = export_track_dimension_transitions(est, cfg)
%EXPORT_TRACK_DIMENSION_TRANSITIONS Export committed 2-D/3-D switch audit.

output_file = cfg_value(cfg, 'dimension_transition_export_file', ...
    fullfile('track_reports', 'track_dimension_transitions.xlsx'));
if isa(output_file, 'string')
    if ~isscalar(output_file)
        error('export_track_dimension_transitions:InvalidPath', ...
            'Excel output path must be scalar text.');
    end
    output_file = char(output_file);
end
if ~ischar(output_file) || isempty(strtrim(output_file))
    error('export_track_dimension_transitions:InvalidPath', ...
        'Excel output path must be nonempty text.');
end
[output_dir, ~, extension] = fileparts(output_file);
if ~strcmpi(extension, '.xlsx')
    error('export_track_dimension_transitions:InvalidExtension', ...
        'Dimension transition output must use the .xlsx extension.');
end
if ~isempty(output_dir) && exist(output_dir, 'dir') ~= 7
    [ok, message] = mkdir(output_dir);
    if ~ok
        error('export_track_dimension_transitions:CreateDirectory', ...
            'Cannot create output directory %s: %s', output_dir, message);
    end
end

records = repmat(struct(), 0, 1);
if isstruct(est) && isfield(est, 'dimension_transition_log') && ...
        isstruct(est.dimension_transition_log)
    records = est.dimension_transition_log;
end
headers = {'序号', '事件序号', '切换时间_s', '切换方向', ...
    '三维公开航迹编号', '二维公开航迹编号', '内部逻辑航迹编号', ...
    '三维内部分支编号', '二维内部分支编号', '切换前模式', '切换后模式', ...
    '判定原因代码', '判定条件', '主动证据命中数', '主动机会窗实际长度', ...
    '升级所需主动命中数M', '升级主动机会窗N', '升级连续判定次数', ...
    '降级连续判定次数', '径向距离标准差_m', '升级径向恢复门_m', ...
    '降级径向门_m', '距离龄期_s', '升级龄期恢复门_s', '降级龄期门_s', ...
    '二维角度95误差_deg', '二维角度95上限_deg', '切换一致性NIS', ...
    '切换一致性门', '二维分支有效', '三维恢复条件满足', '角度投影一致', ...
    '三维外部分支新鲜', '最近可靠距离更新时间_s', '切换提交时间_s'};
rows = cell(numel(records) + 1, numel(headers));
rows(1, :) = headers;
for q = 1:numel(records)
    rows(q + 1, :) = record_row(records(q));
end

temporary_dir = output_dir;
if isempty(temporary_dir), temporary_dir = pwd; end
temporary_file = [tempname(temporary_dir), '.xlsx'];
cleanup = onCleanup(@() delete_if_exists(temporary_file));
write_cells(temporary_file, rows, '维度切换', 'A1');
[moved, message] = movefile(temporary_file, output_file, 'f');
if ~moved
    error('export_track_dimension_transitions:CommitFailed', ...
        'Cannot replace output workbook %s: %s', output_file, message);
end
clear cleanup;

directions = cell(1, numel(records));
if ~isempty(records), directions = {records.direction}; end
n_3d_to_2d = nnz(strcmp(directions, '3D->2D'));
n_2d_to_3d = nnz(strcmp(directions, '2D->3D'));
missing_public_ids = 0;
if ~isempty(records)
    missing_public_ids = nnz(~isfinite([records.public_3d_track_id]) | ...
        ~isfinite([records.public_2d_track_id]));
end
stats = struct();
if isstruct(est) && isfield(est, 'stats') && isstruct(est.stats)
    stats = est.stats;
end
expected_3d_to_2d = numeric_default(stats, 'switch_3d_to_2d_committed', 0);
expected_2d_to_3d = numeric_default(stats, 'switch_2d_to_3d_committed', 0);
unlogged_commits = max(0, expected_3d_to_2d - n_3d_to_2d) + ...
    max(0, expected_2d_to_3d - n_2d_to_3d);
info = struct('file', output_file, 'n_rows', numel(records), ...
    'n_3d_to_2d', n_3d_to_2d, 'n_2d_to_3d', n_2d_to_3d, ...
    'n_incomplete_public_id_rows', missing_public_ids, ...
    'n_manager_commits_missing_from_log', unlogged_commits, ...
    'row_contract', 'one_committed_dimension_switch_per_row', ...
    'excluded_states', 'candidate_pending_blocked_hold_only');
fprintf('升降维事务表已保存: %s（提交切换%d，3D->2D=%d，2D->3D=%d）\n', ...
    output_file, info.n_rows, info.n_3d_to_2d, info.n_2d_to_3d);
if isempty(records)
    fprintf('  本次没有已提交的二维/三维切换；候选、等待确认和阻断事件未写入。\n');
else
    for q = 1:numel(records)
        fprintf(['  [维度切换] t=%.6fs, %s, 3D航迹=%s, 2D航迹=%s, ' ...
            '原因=%s\n'], records(q).t_sec, records(q).direction, ...
            id_text(records(q).public_3d_track_id), ...
            id_text(records(q).public_2d_track_id), records(q).reason_code);
    end
end
if missing_public_ids > 0
    fprintf(['  警告: %d条提交事务未同时形成二维和三维公开ID；' ...
        '表中保留了内部逻辑ID与分支ID供核查。\n'], missing_public_ids);
end
if unlogged_commits > 0
    fprintf(['  警告: 管理器提交计数比事务表多%d条；' ...
        '请用内部transition_log核查该次运行。\n'], unlogged_commits);
end
end

function row = record_row(r)
row = {numeric_or_blank(r, 'sequence'), numeric_or_blank(r, 'event_index'), ...
    numeric_or_blank(r, 't_sec'), text_or_blank(r, 'direction'), ...
    numeric_or_blank(r, 'public_3d_track_id'), ...
    numeric_or_blank(r, 'public_2d_track_id'), ...
    numeric_or_blank(r, 'internal_logical_id'), ...
    numeric_or_blank(r, 'branch_3d_id'), numeric_or_blank(r, 'branch_2d_id'), ...
    text_or_blank(r, 'from_mode'), text_or_blank(r, 'to_mode'), ...
    text_or_blank(r, 'reason_code'), text_or_blank(r, 'condition_text'), ...
    numeric_or_blank(r, 'active_evidence_hits'), ...
    numeric_or_blank(r, 'active_evidence_window'), ...
    numeric_or_blank(r, 'required_active_hits'), ...
    numeric_or_blank(r, 'required_active_window'), ...
    numeric_or_blank(r, 'up_consecutive_required'), ...
    numeric_or_blank(r, 'down_consecutive_required'), ...
    numeric_or_blank(r, 'radial_sigma_m'), ...
    numeric_or_blank(r, 'radial_sigma_warn_m'), ...
    numeric_or_blank(r, 'radial_sigma_drop_m'), ...
    numeric_or_blank(r, 'range_age_s'), numeric_or_blank(r, 'range_age_warn_s'), ...
    numeric_or_blank(r, 'range_age_drop_s'), numeric_or_blank(r, 'angle95_deg'), ...
    numeric_or_blank(r, 'angle95_max_deg'), numeric_or_blank(r, 'switch_nis'), ...
    numeric_or_blank(r, 'switch_gate'), logical_or_blank(r, 'valid2d'), ...
    logical_or_blank(r, 'recover3d'), logical_or_blank(r, 'switch_consistent'), ...
    logical_or_blank(r, 'external_fresh'), ...
    numeric_or_blank(r, 'last_valid_range_t'), ...
    numeric_or_blank(r, 'switch_commit_t')};
end

function write_cells(path, values, sheet, range)
if exist('writecell', 'file') ~= 0
    writecell(values, path, 'Sheet', sheet, 'Range', range);
elseif exist('xlswrite', 'file') ~= 0
    xlswrite(path, values, sheet, range); %#ok<XLSWT>
else
    error('export_track_dimension_transitions:ExcelWriterUnavailable', ...
        'Neither writecell nor xlswrite is available in this MATLAB installation.');
end
end

function delete_if_exists(path)
if exist(path, 'file') == 2, delete(path); end
end

function value = numeric_or_blank(s, name)
value = '';
if isstruct(s) && isfield(s, name) && isscalar(s.(name)) && ...
        isnumeric(s.(name)) && isfinite(s.(name))
    value = double(s.(name));
end
end

function value = logical_or_blank(s, name)
value = '';
if isstruct(s) && isfield(s, name) && isscalar(s.(name)) && ...
        (islogical(s.(name)) || isnumeric(s.(name))) && ...
        isfinite(double(s.(name)))
    value = logical(s.(name));
end
end

function value = text_or_blank(s, name)
value = '';
if ~isstruct(s) || ~isfield(s, name), return; end
candidate = s.(name);
if isa(candidate, 'string') && isscalar(candidate), candidate = char(candidate); end
if ischar(candidate), value = candidate; end
end

function text = id_text(value)
if isscalar(value) && isnumeric(value) && isfinite(value)
    text = sprintf('%.0f', value);
else
    text = '未形成公开ID';
end
end

function value = cfg_value(cfg, name, fallback)
value = fallback;
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
end
end

function value = numeric_default(s, name, fallback)
value = fallback;
if isstruct(s) && isfield(s, name) && isscalar(s.(name)) && ...
        isnumeric(s.(name)) && isfinite(s.(name))
    value = double(s.(name));
end
end
