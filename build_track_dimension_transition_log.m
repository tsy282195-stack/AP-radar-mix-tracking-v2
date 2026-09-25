function log = build_track_dimension_transition_log(manager_log, transactions, ...
        filter_times, output_map, cfg)
%BUILD_TRACK_DIMENSION_TRANSITION_LOG Build committed 2-D/3-D switch records.
% Candidate, warning, blocked and hold-only state changes are deliberately
% excluded. Every retained row is backed by the committed companion owner
% at the event where the formal output dimension changed.

template = transition_template();
log = repmat(template, 0, 1);
if isempty(manager_log) || ~isstruct(manager_log), return; end

seen = cell(1, 0);
for q = 1:numel(manager_log)
    reason = text_field(manager_log(q), 'reason', '');
    [direction, target_dim] = committed_direction(reason);
    if isempty(direction), continue; end

    internal_id = numeric_field(manager_log(q), 'id', NaN);
    switch_t = numeric_field(manager_log(q), 't_sec', NaN);
    [event_index, companion] = find_committed_companion( ...
        transactions, filter_times, switch_t, internal_id, target_dim);
    if isempty(companion), continue; end

    branch_3d_id = numeric_field(companion, 'branch_3d_id', ...
        numeric_field(companion, 'external_3d_id', NaN));
    branch_2d_id = numeric_field(companion, 'branch_2d_id', ...
        numeric_field(companion, 'local_2d_id', NaN));
    public_3d_id = public_track_id( ...
        output_map, internal_id, branch_3d_id, 3);
    public_2d_id = public_track_id( ...
        output_map, internal_id, branch_2d_id, 2);

    duplicate_key = sprintf('%s|%.15g|%.15g|%.15g|%.15g', ...
        direction, switch_t, internal_id, branch_3d_id, branch_2d_id);
    if any(strcmp(seen, duplicate_key)), continue; end
    seen{end + 1} = duplicate_key; %#ok<AGROW>

    quality = struct_field(companion, 'quality', struct());
    item = template;
    item.event_index = event_index;
    item.t_sec = switch_t;
    item.direction = direction;
    item.internal_logical_id = internal_id;
    item.public_3d_track_id = public_3d_id;
    item.public_2d_track_id = public_2d_id;
    item.branch_3d_id = branch_3d_id;
    item.branch_2d_id = branch_2d_id;
    item.from_mode = text_field(manager_log(q), 'from', '');
    item.to_mode = text_field(manager_log(q), 'to', '');
    item.reason_code = reason;
    item.condition_text = condition_description(reason, cfg);
    item.active_evidence_hits = numeric_field( ...
        companion, 'active_evidence_count', NaN);
    item.active_evidence_window = numeric_field( ...
        companion, 'active_evidence_window', ...
        cfg_value(cfg, 'joint_3d_upgrade_N', 3));
    item.required_active_hits = cfg_value(cfg, 'joint_3d_upgrade_M', 2);
    item.required_active_window = cfg_value(cfg, 'joint_3d_upgrade_N', 3);
    item.up_consecutive_required = cfg_value(cfg, 'joint_up_consecutive', 1);
    item.down_consecutive_required = cfg_value(cfg, 'joint_down_consecutive', 3);
    item.radial_sigma_m = numeric_field(quality, 'radial_sigma_m', NaN);
    item.radial_sigma_warn_m = cfg_value(cfg, 'joint_radial_sigma_warn_m', 5000);
    item.radial_sigma_drop_m = cfg_value(cfg, 'joint_radial_sigma_drop_m', 10000);
    item.range_age_s = numeric_field(quality, 'range_age_s', NaN);
    item.range_age_warn_s = cfg_value(cfg, 'joint_range_age_warn_s', 3);
    item.range_age_drop_s = cfg_value(cfg, 'joint_range_age_drop_s', 10);
    item.angle95_deg = numeric_field(quality, 'angle95_deg', NaN);
    item.angle95_max_deg = cfg_value(cfg, 'joint_angle95_max_deg', 1.0);
    item.switch_nis = numeric_field(quality, 'switch_nis', NaN);
    item.switch_gate = cfg_value(cfg, 'joint_switch_gate_2d', 9.2103);
    item.valid2d = logical_field(quality, 'valid2d', false);
    item.recover3d = logical_field(quality, 'recover3d', false);
    item.switch_consistent = logical_field(quality, 'switch_consistent', false);
    item.external_fresh = logical_field(companion, 'external_fresh', false);
    item.last_valid_range_t = numeric_field( ...
        companion, 'last_valid_range_t', NaN);
    item.switch_commit_t = numeric_field(companion, 'switch_commit_t', switch_t);
    log(end + 1, 1) = item; %#ok<AGROW>
end

if isempty(log), return; end
[~, order] = sortrows([[log.event_index].', [log.t_sec].'], [1, 2]);
log = log(order);
for q = 1:numel(log), log(q).sequence = q; end
end

function item = transition_template()
item = struct('sequence', 0, 'event_index', 0, 't_sec', NaN, ...
    'direction', '', 'internal_logical_id', NaN, ...
    'public_3d_track_id', NaN, 'public_2d_track_id', NaN, ...
    'branch_3d_id', NaN, 'branch_2d_id', NaN, ...
    'from_mode', '', 'to_mode', '', 'reason_code', '', ...
    'condition_text', '', 'active_evidence_hits', NaN, ...
    'active_evidence_window', NaN, 'required_active_hits', NaN, ...
    'required_active_window', NaN, 'up_consecutive_required', NaN, ...
    'down_consecutive_required', NaN, 'radial_sigma_m', NaN, ...
    'radial_sigma_warn_m', NaN, 'radial_sigma_drop_m', NaN, ...
    'range_age_s', NaN, 'range_age_warn_s', NaN, ...
    'range_age_drop_s', NaN, 'angle95_deg', NaN, ...
    'angle95_max_deg', NaN, 'switch_nis', NaN, 'switch_gate', NaN, ...
    'valid2d', false, 'recover3d', false, ...
    'switch_consistent', false, 'external_fresh', false, ...
    'last_valid_range_t', NaN, 'switch_commit_t', NaN);
end

function [direction, target_dim] = committed_direction(reason)
direction = ''; target_dim = 0;
if any(strcmp(reason, {'3d_quality_degraded', 'external_3d_lost'}))
    direction = '3D->2D'; target_dim = 2;
elseif any(strcmp(reason, ...
        {'2d_to_3d_2of3_confirmed', '2d_to_3d_quality_confirmed'}))
    direction = '2D->3D'; target_dim = 3;
end
end

function [event_index, companion] = find_committed_companion( ...
        transactions, times, switch_t, internal_id, target_dim)
event_index = 0; companion = [];
if isempty(transactions) || isempty(times) || ...
        ~isfinite(switch_t) || ~isfinite(internal_id)
    return;
end
times = reshape(times, [], 1);
valid = find(isfinite(times));
if isempty(valid), return; end
distance = abs(times(valid) - switch_t);
tolerance = max(1e-9, 64 * eps(max(1, abs(switch_t))));
same_time = valid(distance <= tolerance);
for index = reshape(same_time, 1, [])
    if index > numel(transactions), continue; end
    candidates = transactions{index};
    if isempty(candidates) || ~isstruct(candidates), continue; end
    ids = arrayfun(@(x) numeric_field(x, 'logical_id', NaN), candidates);
    dims = arrayfun(@(x) numeric_field(x, 'active_output_dim', 0), candidates);
    commit_times = arrayfun(@(x) ...
        numeric_field(x, 'switch_commit_t', NaN), candidates);
    committed_now = isfinite(commit_times) & ...
        abs(commit_times - switch_t) <= tolerance;
    match = find(ids == internal_id & dims == target_dim & committed_now, 1);
    if isempty(match), continue; end
    event_index = index;
    companion = candidates(match);
    return;
end
end

function id = public_track_id(records, internal_id, branch_id, dim)
id = NaN;
if isempty(records) || ~isstruct(records), return; end
exact = false(1, numel(records));
for q = 1:numel(records)
    exact(q) = numeric_field(records(q), 'output_dim', 0) == dim && ...
        numeric_field(records(q), 'branch_id', NaN) == branch_id;
end
indices = find(exact);
if numel(indices) == 1
    id = numeric_field(records(indices), 'output_id', NaN);
    return;
end

alias = false(1, numel(records));
for q = 1:numel(records)
    aliases = numeric_vector(records(q), 'internal_logical_ids');
    alias(q) = numeric_field(records(q), 'output_dim', 0) == dim && ...
        any(aliases == internal_id);
end
indices = find(alias);
if numel(indices) == 1
    id = numeric_field(records(indices), 'output_id', NaN);
end
end

function text = condition_description(reason, cfg)
switch reason
    case '3d_quality_degraded'
        text = sprintf(['径向距离1-sigma>=%.6g m且可靠距离龄期>=%.6g s，' ...
            '连续%d次质量评估成立；同时二维角度95%%误差<=%.6g deg，' ...
            '二维/三维角度一致性NIS<=%.6g。'], ...
            cfg_value(cfg, 'joint_radial_sigma_drop_m', 10000), ...
            cfg_value(cfg, 'joint_range_age_drop_s', 10), ...
            cfg_value(cfg, 'joint_down_consecutive', 3), ...
            cfg_value(cfg, 'joint_angle95_max_deg', 1.0), ...
            cfg_value(cfg, 'joint_switch_gate_2d', 9.2103));
    case 'external_3d_lost'
        text = ['成熟三维外部分支已消失；伴随二维角度分支仍有效且已有角度观测，' ...
            '因此由二维分支接管正式输出。'];
    otherwise
        text = sprintf(['最近%d个主动机会中至少%d次有效主动三维命中；' ...
            '空间分支有效，径向距离1-sigma<%.6g m且可靠距离龄期<%.6g s，' ...
            '二维/三维角度一致性NIS<=%.6g。普通二维恢复路径还要求无待定' ...
            '三维重绑定、外部三维分支新鲜，并连续%d次升级判定成立；' ...
            '跨维接管路径在上述证据齐全时直接提交。'], ...
            cfg_value(cfg, 'joint_3d_upgrade_N', 3), ...
            cfg_value(cfg, 'joint_3d_upgrade_M', 2), ...
            cfg_value(cfg, 'joint_radial_sigma_warn_m', 5000), ...
            cfg_value(cfg, 'joint_range_age_warn_s', 3), ...
            cfg_value(cfg, 'joint_switch_gate_2d', 9.2103), ...
            cfg_value(cfg, 'joint_up_consecutive', 1));
end
end

function value = cfg_value(cfg, name, fallback)
value = fallback;
if isstruct(cfg) && isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
end
end

function value = numeric_field(s, name, fallback)
value = fallback;
if isstruct(s) && isfield(s, name) && isscalar(s.(name)) && ...
        isnumeric(s.(name))
    value = double(s.(name));
end
end

function value = logical_field(s, name, fallback)
value = fallback;
if isstruct(s) && isfield(s, name) && isscalar(s.(name)) && ...
        (islogical(s.(name)) || isnumeric(s.(name))) && ...
        isfinite(double(s.(name)))
    value = logical(s.(name));
end
end

function value = text_field(s, name, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, name), return; end
candidate = s.(name);
if isa(candidate, 'string') && isscalar(candidate), candidate = char(candidate); end
if ischar(candidate), value = candidate; end
end

function value = struct_field(s, name, fallback)
value = fallback;
if isstruct(s) && isfield(s, name) && isstruct(s.(name))
    value = s.(name);
end
end

function values = numeric_vector(s, name)
values = zeros(1, 0);
if isstruct(s) && isfield(s, name) && isnumeric(s.(name))
    values = reshape(double(s.(name)), 1, []);
end
end
