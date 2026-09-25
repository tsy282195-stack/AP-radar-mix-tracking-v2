function varargout = joint_review_output(action, varargin)
%JOINT_REVIEW_OUTPUT Pack/read formal output history in compact columnar form.

switch lower(action)
    case 'pack'
        varargout{1} = pack_outputs(varargin{1});
    case 'event'
        varargout{1} = unpack_event(varargin{1}, varargin{2});
    case 'history'
        max_points = inf;
        if numel(varargin) >= 2 && ~isempty(varargin{2}), max_points = varargin{2}; end
        [varargout{1:nargout}] = collect_history(varargin{1}, max_points);
    otherwise
        error('joint_review_output:UnknownAction', '未知操作: %s', action);
end
end

function h = pack_outputs(outputs)
K = numel(outputs);
counts = cellfun(@numel, outputs);
n = sum(counts);
h = struct('format_version', 1, 'storage', 'columnar_by_event', ...
    'n_events', K, 'n_outputs', n, ...
    'event_offsets', zeros(K + 1, 1, 'uint64'), ...
    'event_index', zeros(1, n, 'uint32'), ...
    'id', nan(1, n), 'birth_event', nan(1, n), ...
    'internal_birth_event', nan(1, n), 't_sec', nan(1, n), ...
    'output_dim', zeros(1, n, 'uint8'), 'status_code', zeros(1, n, 'uint8'), ...
    'confirmed', false(1, n), 'formal', true(1, n), ...
    'truth_id', nan(1, n), 'logical_id', nan(1, n), ...
    'internal_logical_id', nan(1, n), 'branch_3d_id', nan(1, n), ...
    'branch_2d_id', nan(1, n), 'active_output_dim', zeros(1, n, 'uint8'), ...
    'last_update_t', nan(1, n), 'az_deg', nan(1, n), ...
    'el_deg', nan(1, n), 'range_m', nan(1, n), ...
    'position_enu', nan(3, n), 'velocity_enu', nan(3, n), ...
    'mode_dictionary', {cell(1, 0)}, 'mode_code', zeros(1, n, 'uint16'), ...
    'switch_dictionary', {cell(1, 0)}, 'switch_code', zeros(1, n, 'uint16'));
p = 0;
for k = 1:K
    h.event_offsets(k) = uint64(p + 1);
    out = outputs{k}; m = numel(out);
    if m == 0, continue; end
    ii = p + (1:m);
    h.event_index(ii) = uint32(k);
    h.id(ii) = field_row(out, 'id', NaN);
    h.birth_event(ii) = field_row(out, 'birth_event', NaN);
    h.internal_birth_event(ii) = field_row(out, 'internal_birth_event', NaN);
    h.t_sec(ii) = field_row(out, 't_sec', NaN);
    h.output_dim(ii) = uint8(field_row(out, 'output_dim', 0));
    h.status_code(ii) = uint8(field_row(out, 'status_code', 0));
    h.confirmed(ii) = logical(field_row(out, 'confirmed', true));
    h.formal(ii) = logical(field_row(out, 'formal', true));
    h.truth_id(ii) = field_row(out, 'truth_id', NaN);
    h.logical_id(ii) = field_row(out, 'logical_id', NaN);
    h.internal_logical_id(ii) = field_row(out, 'internal_logical_id', NaN);
    h.branch_3d_id(ii) = field_row(out, 'branch_3d_id', NaN);
    h.branch_2d_id(ii) = field_row(out, 'branch_2d_id', NaN);
    h.active_output_dim(ii) = uint8(field_row(out, 'active_output_dim', 0));
    h.last_update_t(ii) = field_row(out, 'last_update_t', NaN);
    h.az_deg(ii) = field_row(out, 'az_deg', NaN);
    h.el_deg(ii) = field_row(out, 'el_deg', NaN);
    h.range_m(ii) = field_row(out, 'range_m', NaN);
    h.position_enu(:, ii) = field_matrix(out, 'position_enu', 3);
    h.velocity_enu(:, ii) = field_matrix(out, 'velocity_enu', 3);
    [h.mode_dictionary, h.mode_code(ii)] = encode_text( ...
        h.mode_dictionary, field_texts(out, 'mode'));
    [h.switch_dictionary, h.switch_code(ii)] = encode_text( ...
        h.switch_dictionary, field_texts(out, 'switch_state'));
    p = p + m;
end
h.event_offsets(K + 1) = uint64(p + 1);
end

function out = unpack_event(h, k)
out = repmat(output_template(), 0, 1);
if ~isstruct(h) || ~isfield(h, 'event_offsets') || ...
        k < 1 || k > double(h.n_events)
    return;
end
first = double(h.event_offsets(k)); last = double(h.event_offsets(k + 1)) - 1;
if last < first, return; end
ii = first:last; n = numel(ii); out = repmat(output_template(), n, 1);
for q = 1:n
    j = ii(q);
    out(q).id = h.id(j); out(q).birth_event = h.birth_event(j);
    out(q).internal_birth_event = h.internal_birth_event(j);
    out(q).t_sec = h.t_sec(j); out(q).output_dim = double(h.output_dim(j));
    out(q).status_code = double(h.status_code(j));
    out(q).confirmed = h.confirmed(j); out(q).formal = h.formal(j);
    out(q).truth_id = h.truth_id(j); out(q).logical_id = h.logical_id(j);
    out(q).internal_logical_id = h.internal_logical_id(j);
    out(q).branch_3d_id = h.branch_3d_id(j); out(q).branch_2d_id = h.branch_2d_id(j);
    out(q).active_output_dim = double(h.active_output_dim(j));
    out(q).last_update_t = h.last_update_t(j); out(q).az_deg = h.az_deg(j);
    out(q).el_deg = h.el_deg(j); out(q).range_m = h.range_m(j);
    out(q).position_enu = h.position_enu(:, j);
    out(q).velocity_enu = h.velocity_enu(:, j);
    out(q).mode = decode_text(h.mode_dictionary, h.mode_code(j));
    out(q).switch_state = decode_text(h.switch_dictionary, h.switch_code(j));
end
end

function [ids, H, counts, valid_counts, dims] = collect_history(h, max_points)
template = struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0), 'event_index', [], 'truth_id', []);
ids = zeros(1, 0); H = repmat(template, 0, 1);
counts = zeros(1, 0); valid_counts = zeros(1, 0); dims = zeros(1, 0);
if ~isstruct(h) || ~isfield(h, 'id') || isempty(h.id), return; end
[ids, ~, group] = unique(h.id, 'sorted');
counts = accumarray(group(:), 1, [numel(ids), 1]).';
dims = accumarray(group(:), double(h.output_dim(:)), [numel(ids), 1], @max, 0).';
valid = (h.output_dim == 2 & isfinite(h.az_deg) & isfinite(h.el_deg)) | ...
    (h.output_dim == 3 & all(isfinite(h.position_enu), 1));
valid_counts = accumarray(group(:), double(valid(:)), [numel(ids), 1]).';
H = repmat(template, numel(ids), 1);
for i = 1:numel(ids)
    source = find(group == i);
    n_show = min(numel(source), max_points);
    if n_show < numel(source)
        source = source(unique(round(linspace(1, numel(source), n_show))));
    end
    H(i).t = h.t_sec(source); H(i).az = h.az_deg(source);
    H(i).el = h.el_deg(source); H(i).dim = double(h.output_dim(source));
    H(i).pos = h.position_enu(:, source);
    H(i).event_index = double(h.event_index(source));
    H(i).truth_id = h.truth_id(source);
end
end

function row = field_row(s, name, fallback)
if isfield(s, name), row = reshape([s.(name)], 1, []);
else, row = fallback * ones(1, numel(s));
end
end

function X = field_matrix(s, name, rows)
X = nan(rows, numel(s));
if ~isfield(s, name), return; end
for q = 1:numel(s)
    value = s(q).(name);
    if numel(value) >= rows, X(:, q) = reshape(value(1:rows), rows, 1); end
end
end

function values = field_texts(s, name)
values = repmat({''}, 1, numel(s));
if ~isfield(s, name), return; end
for q = 1:numel(s)
    if ischar(s(q).(name)), values{q} = s(q).(name); end
end
end

function [dictionary, codes] = encode_text(dictionary, values)
codes = zeros(1, numel(values), 'uint16');
[unique_values, ~, local] = unique(values, 'stable');
for q = 1:numel(unique_values)
    slot = find(strcmp(dictionary, unique_values{q}), 1);
    if isempty(slot)
        dictionary{end + 1} = unique_values{q}; %#ok<AGROW>
        slot = numel(dictionary);
    end
    codes(local == q) = uint16(slot);
end
end

function value = decode_text(dictionary, code)
slot = double(code);
if slot >= 1 && slot <= numel(dictionary), value = dictionary{slot};
else, value = '';
end
end

function o = output_template()
o = struct('id', 0, 'birth_event', 0, 'internal_birth_event', 0, ...
    't_sec', NaN, 'mode', '', 'status_code', 0, 'output_dim', 0, ...
    'confirmed', false, 'truth_id', NaN, 'formal', true, ...
    'logical_id', NaN, 'internal_logical_id', NaN, ...
    'branch_3d_id', NaN, 'branch_2d_id', NaN, 'active_output_dim', 0, ...
    'switch_state', '', 'last_update_t', NaN, 'az_deg', NaN, ...
    'el_deg', NaN, 'range_m', NaN, 'position_enu', nan(3, 1), ...
    'velocity_enu', nan(3, 1));
end
