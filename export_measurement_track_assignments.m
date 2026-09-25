function info = export_measurement_track_assignments(est, events, cfg)
%EXPORT_MEASUREMENT_TRACK_ASSIGNMENTS Export auditable input-to-track routing.
% One input measurement occupies one row.  Active RAE inputs populate
% columns 2-4; physical-passive AE and active AE-only inputs populate
% columns 5-7.  Empty track cells mean the measurement was not retained by
% either filter branch.  Repeated times are intentional.

output_file = get_cfg(cfg, 'measurement_assignment_export_file', ...
    fullfile('track_reports', 'measurement_track_assignment.xlsx'));
if isa(output_file, 'string')
    if ~isscalar(output_file)
        error('export_measurement_track_assignments:InvalidPath', ...
            'Excel output path must be scalar text.');
    end
    output_file = char(output_file);
end
if ~ischar(output_file) || isempty(strtrim(output_file))
    error('export_measurement_track_assignments:InvalidPath', ...
        'Excel output path must be nonempty text.');
end

[output_dir, ~, extension] = fileparts(output_file);
if ~strcmpi(extension, '.xlsx')
    error('export_measurement_track_assignments:InvalidExtension', ...
        'Measurement assignment output must use the .xlsx extension.');
end
if ~isempty(output_dir) && exist(output_dir, 'dir') ~= 7
    [ok, message] = mkdir(output_dir);
    if ~ok
        error('export_measurement_track_assignments:CreateDirectory', ...
            'Cannot create output directory %s: %s', output_dir, message);
    end
end
temporary_dir = output_dir;
if isempty(temporary_dir), temporary_dir = pwd; end
temporary_file = [tempname(temporary_dir), '.xlsx'];
temporary_cleanup = onCleanup(@() delete_if_exists(temporary_file));

headers = {'时间_s', '三维量测目标编号', '三维量测对应三维航迹编号', ...
    '三维量测对应二维航迹编号', '二维量测目标编号', ...
    '二维量测对应三维航迹编号', '二维量测对应二维航迹编号'};
writer = init_writer(temporary_file, headers);
n_3d = 0; n_2d = 0; n_assigned = 0;

for k = 1:numel(events)
    event = events(k);
    na = measurement_count(event, 'active');
    active_ids = measurement_values(event, 'active', 'ids', na, NaN);
    active_times = measurement_values(event, 'active', 't_sec', na, event.t_sec);
    active_has_range = measurement_values( ...
        event, 'active', 'has_range', na, true);
    active_has_range = isfinite(active_has_range) & active_has_range ~= 0;
    for mi = 1:na
        row = repmat({''}, 1, 7);
        row{1} = finite_or_blank(active_times(mi));
        [track_3d_id, track_2d_id, associated] = measurement_assignment( ...
            est, k, 'active', mi);
        if active_has_range(mi)
            row{2} = finite_or_blank(active_ids(mi));
            row{3} = track_or_blank(track_3d_id);
            row{4} = track_or_blank(track_2d_id);
            n_3d = n_3d + 1;
        else
            row{5} = finite_or_blank(active_ids(mi));
            row{6} = track_or_blank(track_3d_id);
            row{7} = track_or_blank(track_2d_id);
            n_2d = n_2d + 1;
        end
        writer = append_row(writer, row);
        n_assigned = n_assigned + double(associated);
    end

    np = measurement_count(event, 'passive');
    passive_ids = measurement_values(event, 'passive', 'ids', np, NaN);
    passive_times = measurement_values(event, 'passive', 't_sec', np, event.t_sec);
    for mi = 1:np
        row = repmat({''}, 1, 7);
        row{1} = finite_or_blank(passive_times(mi));
        row{5} = finite_or_blank(passive_ids(mi));
        [track_3d_id, track_2d_id, associated] = measurement_assignment( ...
            est, k, 'passive', mi);
        row{6} = track_or_blank(track_3d_id);
        row{7} = track_or_blank(track_2d_id);
        writer = append_row(writer, row);
        n_2d = n_2d + 1;
        n_assigned = n_assigned + double(associated);
    end
end
writer = flush_writer(writer);
[moved, move_message] = movefile(temporary_file, output_file, 'f');
if ~moved
    error('export_measurement_track_assignments:CommitFailed', ...
        'Cannot replace output workbook %s: %s', output_file, move_message);
end
clear temporary_cleanup;

info = struct('file', output_file, 'n_rows', n_3d + n_2d, ...
    'n_active_3d_measurements', n_3d, 'n_2d_measurements', n_2d, ...
    'n_assigned', n_assigned, ...
    'n_unassigned', n_3d + n_2d - n_assigned, ...
    'n_sheets', writer.sheet_index, ...
    'row_contract', 'one_input_measurement_per_row', ...
    'two_d_definition', 'physical_passive_AE_and_active_AE_only', ...
    'track_id_contract', ...
    'positive_formal_output_id;negative_associated_nonformal_branch');
fprintf(['量测-航迹对应表已保存: %s（%d行，%d个工作表；' ...
    '三维量测%d，二维量测%d）\n'], output_file, info.n_rows, ...
    info.n_sheets, n_3d, n_2d);
fprintf('  航迹编号口径：正数=正式公开航迹ID，负数=已关联但未形成正式输出的内部分支。\n');
end

function writer = init_writer(path, headers)
writer = struct('path', path, 'headers', {headers}, ...
    'buffer', {cell(5000, numel(headers))}, 'buffer_count', 0, ...
    'sheet_index', 1, 'sheet_data_rows', 0, 'next_excel_row', 2, ...
    'max_data_rows', 1048575);
write_cells(path, headers, sheet_name(writer.sheet_index), 'A1');
end

function writer = append_row(writer, row)
if writer.sheet_data_rows >= writer.max_data_rows
    writer = flush_writer(writer);
    writer.sheet_index = writer.sheet_index + 1;
    writer.sheet_data_rows = 0;
    writer.next_excel_row = 2;
    write_cells(writer.path, writer.headers, ...
        sheet_name(writer.sheet_index), 'A1');
end
writer.buffer_count = writer.buffer_count + 1;
writer.buffer(writer.buffer_count, :) = row;
writer.sheet_data_rows = writer.sheet_data_rows + 1;
if writer.buffer_count >= size(writer.buffer, 1)
    writer = flush_writer(writer);
end
end

function writer = flush_writer(writer)
if writer.buffer_count <= 0, return; end
payload = writer.buffer(1:writer.buffer_count, :);
write_cells(writer.path, payload, sheet_name(writer.sheet_index), ...
    sprintf('A%d', writer.next_excel_row));
writer.next_excel_row = writer.next_excel_row + writer.buffer_count;
writer.buffer_count = 0;
writer.buffer(:) = {[]};
end

function write_cells(path, values, sheet, range)
if exist('writecell', 'file') ~= 0
    writecell(values, path, 'Sheet', sheet, 'Range', range);
elseif exist('xlswrite', 'file') ~= 0
    xlswrite(path, values, sheet, range); %#ok<XLSWT>
else
    error('export_measurement_track_assignments:ExcelWriterUnavailable', ...
        'Neither writecell nor xlswrite is available in this MATLAB installation.');
end
end

function delete_if_exists(path)
if exist(path, 'file') == 2
    delete(path);
end
end

function name = sheet_name(index)
name = sprintf('关联_%03d', index);
end

function n = measurement_count(event, type)
n = 0;
if ~isstruct(event) || ~isfield(event, type) || ~isstruct(event.(type)), return; end
data = event.(type);
if isfield(data, 'n_meas') && isscalar(data.n_meas) && isfinite(data.n_meas)
    n = max(0, round(data.n_meas));
elseif strcmp(type, 'active') && isfield(data, 'rae')
    n = size(data.rae, 2);
elseif strcmp(type, 'passive') && isfield(data, 'ang')
    n = size(data.ang, 2);
end
end

function values = measurement_values(event, type, field, n, fallback)
values = fallback * ones(1, n);
if n == 0 || ~isfield(event, type) || ~isfield(event.(type), field), return; end
source = reshape(event.(type).(field), 1, []);
m = min(n, numel(source));
values(1:m) = source(1:m);
end

function [track_3d_id, track_2d_id, associated] = measurement_assignment( ...
        est, event_index, type, measurement_index)
track_3d_id = NaN; track_2d_id = NaN; associated = false;
if ~isfield(est, 'assoc') || event_index > numel(est.assoc) || ...
        isempty(est.assoc{event_index})
    return;
end
a = est.assoc{event_index};
matches = false(1, numel(a.id));
for q = 1:numel(a.id)
    matches(q) = association_type_matches( ...
        indexed_text(a, 'type', q, ''), type) && ...
        indexed_number(a, 'meas_index', q, NaN) == measurement_index;
end

indices = find(matches);
if numel(indices) > 1
    error('export_measurement_track_assignments:DuplicateAssociation', ...
        'Event %d %s measurement %d has multiple associated tracks.', ...
        event_index, type, measurement_index);
elseif isempty(indices)
    return;
end
q = indices(1);
filter_dim = indexed_number(a, 'filter_dim', q, 0);
track_3d_id = indexed_number(a, 'public_3d_track_id', q, NaN);
track_2d_id = indexed_number(a, 'public_2d_track_id', q, NaN);
selected_id = indexed_number(a, 'id', q, NaN);
if ~isfinite(track_3d_id) && filter_dim == 3, track_3d_id = selected_id; end
if ~isfinite(track_2d_id) && filter_dim == 2, track_2d_id = selected_id; end
associated = valid_track_id(track_3d_id) || valid_track_id(track_2d_id);
end

function tf = association_type_matches(value, requested)
if isa(value, 'string') && isscalar(value), value = char(value); end
tf = ischar(value) && ~isempty(strfind(value, requested)); %#ok<STREMP>
end

function value = indexed_number(s, field, index, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, field), return; end
source = s.(field);
if numel(source) >= index, value = double(source(index)); end
end

function value = indexed_text(s, field, index, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, field), return; end
source = s.(field);
if iscell(source) && numel(source) >= index && ischar(source{index})
    value = source{index};
end
end

function value = finite_or_blank(value)
if ~isscalar(value) || ~isfinite(value), value = ''; end
end

function value = track_or_blank(value)
if ~valid_track_id(value), value = ''; end
end

function tf = valid_track_id(value)
tf = isscalar(value) && isfinite(value) && value ~= 0;
end

function value = get_cfg(cfg, name, fallback)
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
else
    value = fallback;
end
end
