function info = view_joint_tracks(est, events, opts)
%VIEW_JOINT_TRACKS Inspect selected dimension-scoped formal output tracks.

if nargin < 3, opts = struct(); end
[ids, H] = collect_history(est);
requested = get_opt(opts, 'track_ids', []);
if isempty(requested), selected = ids; else, selected = intersect(requested(:).', ids, 'stable'); end
if isempty(selected)
    error('view_joint_tracks:NoTrack', '所选航迹不存在。可用ID: %s', mat2str(ids));
end
show_ref = get_opt(opts, 'show_reference', true);
show_angle = get_opt(opts, 'show_angle', true);
show_enu = get_opt(opts, 'show_enu', true);
view_cfg = get_opt(opts, 'cfg', struct());
az_center = get_opt(view_cfg, 'plot_azimuth_center_deg', 0);
ref_data = prepare_reference_data(est, events, H, ids, selected, opts, show_ref);
colors = lines(numel(selected));
selected_has2 = false(1, numel(selected));
selected_has3 = false(1, numel(selected));
for s = 1:numel(selected)
    h = H(ids == selected(s));
    selected_has2(s) = any(angle_mode_mask(h));
    selected_has3(s) = any(position_mode_mask(h));
end

fprintf('分维正式输出航迹可用ID: %s\n', mat2str(ids));
fprintf('当前查看ID: %s\n', mat2str(selected));

if show_angle && any(selected_has2)
    figure('Name', '选中二维纯角度航迹 AE', 'Position', [80, 90, 900, 700]);
    hold on; grid on; box on;
    if show_ref, plot_full_angle_truth(ref_data, 1.4, az_center); end
    for s = 1:numel(selected)
        if ~selected_has2(s), continue; end
        h = H(ids == selected(s));
        [az, el] = angle_mode_series(h, az_center);
        plot(az, el, '.-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, [az; el], selected(s), colors(s, :));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)'); title('选中二维纯角度航迹');
    xlim(az_center + [-180, 180]);
    legend('Location', 'bestoutside'); hold off;

    figure('Name', '二维角度参考对比', 'Position', [120, 120, 980, 720]);
    for ax = 1:2
        subplot(2, 1, ax); hold on; grid on; box on;
        if show_ref, plot_full_angle_truth_axis(ref_data, ax, az_center); end
        for s = 1:numel(selected)
            if ~selected_has2(s), continue; end
            h = H(ids == selected(s));
            valid = angle_mode_mask(h);
            val = h.az; if ax == 1, val = azimuth_for_plot(val, az_center); else, val = h.el; end
            val(~valid) = NaN;
            plot_val = unwrap_for_plot(val);
            plot(h.t, plot_val, '-', 'Color', colors(s, :), 'LineWidth', 1.3, ...
                'DisplayName', sprintf('Track %d 滤波', selected(s)));
            annotate_track_start(gca, [h.t; plot_val], selected(s), colors(s, :));
        end
        ylabel(axis_name(ax)); if ax == 1, title('二维纯角度滤波与完整真值'); end
        if ax == 2, xlabel('Time (s)'); end
        legend('Location', 'bestoutside'); hold off;
    end

    if show_ref
        figure('Name', '二维角度误差', 'Position', [150, 145, 980, 720]);
        for ax = 1:2
            subplot(2, 1, ax); hold on; grid on; box on;
            for s = 1:numel(selected)
                if ~selected_has2(s), continue; end
                h = H(ids == selected(s));
                valid = angle_mode_mask(h);
                ref = ref_data(ids == selected(s)).angle;
                ref(:, ~valid) = NaN;
                err = nan(1, numel(h.t));
                if ax == 1
                    err(valid) = angle_diff(h.az(valid), ref(1, valid));
                else
                    err(valid) = h.el(valid) - ref(2, valid);
                end
                plot(h.t, err, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
                    'DisplayName', sprintf('Track %d', selected(s)));
                annotate_track_start(gca, [h.t; err], selected(s), colors(s, :));
            end
            yline(0, 'k:'); ylabel([axis_name(ax), '误差']);
            if ax == 1, title('二维角度误差'); end
            if ax == 2, xlabel('Time (s)'); end
            legend('Location', 'bestoutside'); hold off;
        end
    end
end

if show_enu && any(selected_has3)
    figure('Name', '选中三维主动航迹 ENU', 'Position', [180, 170, 940, 720]);
    hold on; grid on; box on;
    if show_ref, plot_full_position_truth(ref_data, 1.4); end
    for s = 1:numel(selected)
        if ~selected_has3(s), continue; end
        h = H(ids == selected(s));
        pos = position_mode_series(h);
        plot3(pos(1, :), pos(2, :), pos(3, :), '.-', ...
            'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, pos, selected(s), colors(s, :));
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title('选中三维主动空间航迹'); view(45, 30); axis equal;
    legend('Location', 'bestoutside'); hold off;

    plot_enu_comparison(ids, H, selected, colors, show_ref, ref_data);
end

selected_ref = ref_data(ismember(ids, selected));
info = struct('available_ids', ids, 'selected_ids', selected, 'history', H, ...
    'angle_ids', selected(selected_has2), 'enu_ids', selected(selected_has3), ...
    'reference', selected_ref, 'truth_summary', reference_summary(selected_ref));
end

function plot_enu_comparison(ids, H, selected, colors, show_ref, ref_data)
names = {'East (m)', 'North (m)', 'Up (m)'};
figure('Name', 'ENU参考对比', 'Position', [210, 190, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    if show_ref, plot_full_position_truth_axis(ref_data, ax); end
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        val = h.pos(ax, :); val(~valid) = NaN;
        plot(h.t, val, '-', 'Color', colors(s, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('Track %d 滤波', selected(s)));
        annotate_track_start(gca, [h.t; val], selected(s), colors(s, :));
    end
    ylabel(names{ax}); if ax == 1, title('三维ENU滤波与完整真值'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
if ~show_ref, return; end

figure('Name', 'ENU误差', 'Position', [240, 215, 1020, 800]);
for ax = 1:3
    subplot(3, 1, ax); hold on; grid on; box on;
    for s = 1:numel(selected)
        h = H(ids == selected(s)); valid = position_mode_mask(h);
        if ~any(valid), continue; end
        ref = ref_data(ids == selected(s)).position;
        ref(:, ~valid) = NaN;
        err = nan(1, numel(h.t));
        err(valid) = h.pos(ax, valid) - ref(ax, valid);
        plot(h.t, err, '-', 'Color', colors(s, :), ...
            'LineWidth', 1.2, 'DisplayName', sprintf('Track %d', selected(s)));
        annotate_track_start(gca, [h.t; err], selected(s), colors(s, :));
    end
    yline(0, 'k:'); ylabel([names{ax}, '误差']);
    if ax == 1, title('三维ENU误差'); end
    if ax == 3, xlabel('Time (s)'); end
    legend('Location', 'bestoutside'); hold off;
end
end

function valid = angle_mode_mask(h)
valid = h.dim == 2 & isfinite(h.az) & isfinite(h.el);
end

function [az, el] = angle_mode_series(h, center_deg)
valid = angle_mode_mask(h);
az = azimuth_for_plot(h.az, center_deg); el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_wrap(az, el);
end

function valid = position_mode_mask(h)
valid = h.dim == 3 & all(isfinite(h.pos), 1);
end

function pos = position_mode_series(h)
valid = position_mode_mask(h);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, H] = collect_history(est)
if ~isfield(est, 'output') && isfield(est, 'output_history')
    [ids, H] = joint_review_output('history', est.output_history, inf);
    return;
end
ids = zeros(1, 0);
H = repmat(struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0), 'event_index', [], 'truth_id', []), 0, 1);
for k = 1:numel(est.output)
    out = est.output{k};
    for q = 1:numel(out)
        i = find(ids == out(q).id, 1);
        if isempty(i)
            ids(end + 1) = out(q).id; %#ok<AGROW>
            H(end + 1, 1) = struct('t', [], 'az', [], 'el', [], 'dim', [], ...
                'pos', zeros(3, 0), 'event_index', [], 'truth_id', []); %#ok<AGROW>
            i = numel(ids);
        end
        H(i).t(end + 1) = out(q).t_sec; H(i).az(end + 1) = out(q).az_deg;
        H(i).el(end + 1) = out(q).el_deg; H(i).dim(end + 1) = out(q).output_dim;
        H(i).pos(:, end + 1) = out(q).position_enu; H(i).event_index(end + 1) = k;
        H(i).truth_id(end + 1) = out(q).truth_id;
    end
end
end

function data = prepare_reference_data(est, events, H, ids, selected, opts, enabled)
template = struct('track_id', 0, 'truth_key', NaN, 'truth_label', '', ...
    'angle', zeros(2, 0), 'position', zeros(3, 0), ...
    'n_angle', 0, 'n_position', 0, 'touched_truth_keys', zeros(1, 0), ...
    'truth_tracks', repmat(empty_truth_track(), 1, 0));
data = repmat(template, numel(ids), 1);
for i = 1:numel(ids)
    data(i).track_id = ids(i);
    data(i).angle = nan(2, numel(H(i).t));
    data(i).position = nan(3, numel(H(i).t));
end
if ~enabled || isempty(events) || isempty(ids), return; end
cfg = get_opt(opts, 'cfg', struct());
if isfield(opts, 'truth_use_split') && ~isempty(opts.truth_use_split)
    cfg.truth_id_split_enabled = logical(opts.truth_use_split);
end
metrics = get_opt(opts, 'metrics', struct());
if ~valid_evaluation_metrics(metrics, cfg)
    cfg_eval = cfg;
    cfg_eval.metrics_max_print = 0;
    cfg_eval.metrics_progress_enabled = false;
    metrics = evaluate_joint_tracking_metrics(est, events, cfg_eval);
end
labels = build_joint_truth_labels(events, cfg);
details = metrics.track_details;
truth_tracks = repmat(empty_truth_track(), 1, 0);
select_mode = lower(char(get_opt(opts, 'truth_select_mode', 'dominant')));
if strcmp(select_mode, 'all_touch'), select_mode = 'all_touched'; end
if ~any(strcmp(select_mode, {'dominant', 'matched', 'all_touched'}))
    error('view_joint_tracks:InvalidTruthSelectMode', ...
        'truth_select_mode必须为dominant/matched/all_touch/all_touched。');
end
for i = 1:numel(ids)
    if ~ismember(ids(i), selected), continue; end
    q = find([details.track_id] == ids(i), 1);
    if isempty(q), key = NaN; dim = history_dimension(H(i));
    else
        dim = history_dimension(H(i));
        key = detail_reference_for_dimension(details(q), dim);
    end
    data(i).truth_key = key;
    if isfinite(key) && ismember(dim, [2, 3])
        track = scoped_truth_track(est, events, labels, key, dim);
        track.role = 'primary';
        slot = find([truth_tracks.tid] == key & [truth_tracks.scope_dim] == dim, 1);
        if isempty(slot)
            truth_tracks(end + 1) = track; %#ok<AGROW>
        else
            truth_tracks(slot).role = 'primary';
        end
        data(i).truth_label = track.label;
        data(i).angle = interpolate_angle_series( ...
            track.angle_t, track.angle, H(i).t);
        data(i).position = interpolate_linear_series( ...
            track.t, track.p, H(i).t);
    end
    data(i).n_angle = nnz(all(isfinite(data(i).angle), 1));
    data(i).n_position = nnz(all(isfinite(data(i).position), 1));
    if data(i).n_angle == 0 && data(i).n_position == 0
        fprintf(2, ['[view_track] Track %d 在%dD评价作用域中没有一对一匹配参考，' ...
            '不再使用局部多数投票回退。\n'], ids(i), dim);
    end
    if strcmp(select_mode, 'all_touched')
        touched = touched_truth_keys(est, labels, ids(i), dim);
        data(i).touched_truth_keys = touched;
        for touched_key = touched
            slot = find([truth_tracks.tid] == touched_key & ...
                [truth_tracks.scope_dim] == dim, 1);
            if isempty(slot)
                track = scoped_truth_track(est, events, labels, touched_key, dim);
                track.role = 'touched';
                if ~isempty(track.angle_t) || ~isempty(track.t)
                    truth_tracks(end + 1) = track; %#ok<AGROW>
                end
            end
        end
    end
end
if logical(get_opt(opts, 'truth_all', false))
    selected_dims = zeros(1, 0);
    for i = find(ismember(ids, selected))
        selected_dims(end + 1) = history_dimension(H(i)); %#ok<AGROW>
    end
    keys = reshape(labels.summary.instance_labels, 1, []);
    for dim = unique(selected_dims)
        for key = keys
            if any([truth_tracks.tid] == key & [truth_tracks.scope_dim] == dim), continue; end
            track = scoped_truth_track(est, events, labels, key, dim);
            track.role = 'all';
            if ~isempty(track.angle_t) || ~isempty(track.t)
                truth_tracks(end + 1) = track; %#ok<AGROW>
            end
        end
    end
end
for i = 1:numel(data), data(i).truth_tracks = truth_tracks; end
fprintf(['[view_track] 主参考=评价器二维/三维独立一对一匹配；' ...
    '显示模式=%s，附加真值仅用于混批诊断，不参与误差计算。\n'], select_mode);
end

function keys = touched_truth_keys(est, labels, track_id, dim)
keys = zeros(1, 0);
if ~isfield(est, 'assoc') || isempty(est.assoc), return; end
for k = 1:min(numel(est.assoc), numel(labels.active))
    a = est.assoc{k};
    if isempty(a) || ~isfield(a, 'id'), continue; end
    for q = find(reshape(a.id, 1, []) == track_id)
        filter_dim = dim;
        if isfield(a, 'filter_dim') && q <= numel(a.filter_dim) && ...
                ismember(a.filter_dim(q), [2, 3])
            filter_dim = a.filter_dim(q);
        end
        if filter_dim ~= dim, continue; end
        type = '';
        if isfield(a, 'type') && q <= numel(a.type), type = a.type{q}; end
        mi = 0;
        if isfield(a, 'meas_index') && q <= numel(a.meas_index)
            mi = round(a.meas_index(q));
        end
        key = NaN;
        if mi >= 1 && is_active_association_type(type) && ...
                mi <= numel(labels.active{k})
            key = labels.active{k}(mi);
        elseif mi >= 1 && is_passive_association_type(type) && ...
                k <= numel(labels.passive) && mi <= numel(labels.passive{k})
            key = labels.passive{k}(mi);
        elseif isfield(a, 'tid') && q <= numel(a.tid)
            key = a.tid(q);
        end
        if isfinite(key), keys(end + 1) = key; end %#ok<AGROW>
    end
end
keys = unique(keys, 'stable');
end

function tf = valid_evaluation_metrics(metrics, cfg)
tf = isstruct(metrics) && isfield(metrics, 'evaluation_version') && ...
    metrics.evaluation_version >= 7 && isfield(metrics, 'track_details') && ...
    isstruct(metrics.track_details);
if ~tf || ~isfield(cfg, 'truth_id_split_enabled') || ...
        ~isfield(metrics, 'truth_targets') || ...
        ~isfield(metrics.truth_targets, 'enabled')
    return;
end
tf = logical(metrics.truth_targets.enabled) == logical(cfg.truth_id_split_enabled);
end

function dim = history_dimension(history)
dims = unique(history.dim(ismember(history.dim, [2, 3])));
if numel(dims) ~= 1
    error('view_joint_tracks:MixedDimensionOutputId', ...
        '公开航迹ID必须只属于二维或三维一个输出域。');
end
dim = dims(1);
end

function key = detail_reference_for_dimension(detail, dim)
key = NaN;
if dim == 3 && isfield(detail, 'three_d')
    key = detail.three_d.matched_truth_id;
elseif dim == 2 && isfield(detail, 'two_d')
    key = detail.two_d.matched_truth_id;
end
end

function track = scoped_truth_track(est, events, labels, key, dim)
track = empty_truth_track();
track.tid = key; track.scope_dim = dim;
track.label = scoped_truth_label(labels.summary, key, dim);
for k = 1:numel(events)
    e = events(k);
    if dim == 3
        n = e.active.n_meas;
        ids = sized_row_local(labels.active{k}, n, NaN);
        has_range = false(1, n);
        m = min(n, numel(e.active.has_range));
        if m > 0, has_range(1:m) = logical(e.active.has_range(1:m)); end
        keep = find(ids == key & has_range);
        keep = keep(keep <= size(e.active.rae, 2));
        times = measurement_times_local(e.active.t_sec, n, e.t_sec);
        for q = keep
            if size(e.active.rae, 1) >= 3 && all(isfinite(e.active.rae(2:3, q)))
                track.angle_t(end + 1) = times(q); %#ok<AGROW>
                track.angle(:, end + 1) = e.active.rae(2:3, q); %#ok<AGROW>
            end
            if q <= size(e.active.xyz, 2) && all(isfinite(e.active.xyz(1:3, q)))
                track.t(end + 1) = times(q); %#ok<AGROW>
                track.p(:, end + 1) = e.active.xyz(1:3, q); %#ok<AGROW>
            end
        end
    else
        % Legacy/source events may keep range-invalid active measurements in
        % the active container. Include those angle-only references as well
        % as current kind=2 measurements carried by the passive container.
        n = e.active.n_meas;
        ids = sized_row_local(labels.active{k}, n, NaN);
        has_range = false(1, n);
        m = min(n, numel(e.active.has_range));
        if m > 0, has_range(1:m) = logical(e.active.has_range(1:m)); end
        routed = active_2d_route_mask(est, k, n);
        keep = find(ids == key & ~has_range & routed);
        keep = keep(keep <= size(e.active.rae, 2));
        times = measurement_times_local(e.active.t_sec, n, e.t_sec);
        for q = keep
            if size(e.active.rae, 1) >= 3 && all(isfinite(e.active.rae(2:3, q)))
                track.angle_t(end + 1) = times(q); %#ok<AGROW>
                track.angle(:, end + 1) = e.active.rae(2:3, q); %#ok<AGROW>
            end
        end

        n = e.passive.n_meas;
        ids = sized_row_local(labels.passive{k}, n, NaN);
        routed = passive_2d_route_mask(est, k, n);
        keep = find(ids == key & routed);
        keep = keep(keep <= size(e.passive.ang, 2));
        times = measurement_times_local(e.passive.t_sec, n, e.t_sec);
        for q = keep
            if size(e.passive.ang, 1) >= 2 && all(isfinite(e.passive.ang(1:2, q)))
                track.angle_t(end + 1) = times(q); %#ok<AGROW>
                track.angle(:, end + 1) = e.passive.ang(1:2, q); %#ok<AGROW>
            end
        end
    end
end
[track.angle_t, track.angle] = merge_angle_reference(track.angle_t, track.angle);
[track.t, track.p] = merge_linear_reference(track.t, track.p);
end

function routed = active_2d_route_mask(est, k, n)
routed = false(1, n);
if n == 0, return; end
if isfield(est, 'measurement_disposition') && ...
        k <= numel(est.measurement_disposition) && ...
        ~isempty(est.measurement_disposition{k})
    d = est.measurement_disposition{k};
    if isfield(d, 'active') && isfield(d.active, 'input_dim')
        values = reshape(d.active.input_dim, 1, []);
        m = min(n, numel(values)); routed(1:m) = values(1:m) == 2;
        return;
    end
end
if ~isfield(est, 'assoc') || k > numel(est.assoc) || isempty(est.assoc{k}), return; end
a = est.assoc{k};
for q = 1:numel(a.id)
    if ~isfield(a, 'type') || q > numel(a.type) || ...
            ~is_active_association_type(a.type{q}), continue; end
    if ~isfield(a, 'meas_index') || q > numel(a.meas_index), continue; end
    mi = a.meas_index(q); dim = association_input_dim(a, q);
    if mi >= 1 && mi <= n && mi == round(mi) && dim == 2, routed(mi) = true; end
end
end

function tf = is_active_association_type(type)
tf = ischar(type) && ~isempty(strfind(type, 'active')); %#ok<STREMP>
end

function tf = is_passive_association_type(type)
tf = ischar(type) && ~isempty(strfind(type, 'passive')); %#ok<STREMP>
end

function label = scoped_truth_label(summary, key, dim)
label = sprintf('%dD truth %g', dim, key);
if ~isstruct(summary) || ~isfield(summary, 'instance_labels'), return; end
q = find(summary.instance_labels == key, 1);
if ~isempty(q) && isfield(summary, 'instance_label_text') && ...
        q <= numel(summary.instance_label_text)
    label = sprintf('%dD %s', dim, summary.instance_label_text{q});
end
end

function routed = passive_2d_route_mask(est, k, n)
routed = false(1, n);
if n == 0, return; end
if isfield(est, 'measurement_disposition') && ...
        k <= numel(est.measurement_disposition) && ...
        ~isempty(est.measurement_disposition{k})
    d = est.measurement_disposition{k};
    if isfield(d, 'passive') && isfield(d.passive, 'input_dim')
        values = reshape(d.passive.input_dim, 1, []);
        m = min(n, numel(values)); routed(1:m) = values(1:m) == 2;
        return;
    end
end
if ~isfield(est, 'assoc') || k > numel(est.assoc) || isempty(est.assoc{k}), return; end
a = est.assoc{k};
for q = 1:numel(a.id)
    if ~isfield(a, 'type') || q > numel(a.type) || ...
            ~is_passive_association_type(a.type{q}), continue; end
    if ~isfield(a, 'meas_index') || q > numel(a.meas_index), continue; end
    mi = a.meas_index(q); dim = association_input_dim(a, q);
    if mi >= 1 && mi <= n && mi == round(mi) && dim == 2, routed(mi) = true; end
end
end

function dim = association_input_dim(a, q)
dim = 0;
if isfield(a, 'input_dim') && q <= numel(a.input_dim), dim = a.input_dim(q); end
if ~ismember(dim, [2, 3]) && isfield(a, 'filter_dim') && q <= numel(a.filter_dim)
    dim = a.filter_dim(q);
end
end

function x = sized_row_local(x0, n, fill)
x = fill * ones(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0)); x(1:m) = reshape(x0(1:m), 1, []);
end

function t = measurement_times_local(t0, n, fallback)
t = fallback * ones(1, n);
if n == 0 || isempty(t0), return; end
if isscalar(t0), t(:) = t0; else, m = min(n, numel(t0)); t(1:m) = t0(1:m); end
end

function [t, z] = merge_angle_reference(t, z)
if isempty(t), t = zeros(1, 0); z = zeros(2, 0); return; end
[t, order] = sort(t(:).'); z = z(:, order); [u, ~, group] = unique(t, 'stable');
merged = nan(2, numel(u));
for i = 1:numel(u)
    q = group == i;
    merged(:, i) = [atan2d(mean(sind(z(1, q))), mean(cosd(z(1, q)))); ...
        mean(z(2, q))];
end
t = u; z = merged;
end

function [t, z] = merge_linear_reference(t, z)
if isempty(t), t = zeros(1, 0); z = zeros(size(z, 1), 0); return; end
[t, order] = sort(t(:).'); z = z(:, order); [u, ~, group] = unique(t, 'stable');
merged = nan(size(z, 1), numel(u));
for i = 1:numel(u), merged(:, i) = mean(z(:, group == i), 2); end
t = u; z = merged;
end

function ref = interpolate_angle_series(t, z, query)
ref = nan(2, numel(query));
if isempty(t), return; end
if numel(t) == 1
    ref(:, abs(query - t) <= time_tolerance(t)) = z(:, 1);
    return;
end
az = unwrap(deg2rad(z(1, :)));
ref(1, :) = rad2deg(interp1(t, az, query, 'linear', NaN));
ref(1, :) = mod(ref(1, :) + 180, 360) - 180;
ref(2, :) = interp1(t, z(2, :), query, 'linear', NaN);
end

function ref = interpolate_linear_series(t, z, query)
ref = nan(size(z, 1), numel(query));
if isempty(t), return; end
if numel(t) == 1
    ref(:, abs(query - t) <= time_tolerance(t)) = z(:, 1);
    return;
end
for row = 1:size(z, 1)
    ref(row, :) = interp1(t, z(row, :), query, 'linear', NaN);
end
end

function plot_full_angle_truth(data, line_width, center_deg)
tracks = selected_truth_tracks(data);
if ~isempty(tracks), tracks = tracks([tracks.scope_dim] == 2); end
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).angle_t), continue; end
    az0 = azimuth_for_plot(tracks(i).angle(1, :), center_deg);
    [az, el] = break_wrap(az0, tracks(i).angle(2, :));
    plot(az, el, 'o-', 'Color', colors(i, :), 'LineWidth', line_width, ...
        'MarkerSize', 3.5, ...
        'DisplayName', truth_display_name(tracks(i)));
    annotate_truth_start(gca, [az; el], tracks(i).label, colors(i, :));
end
end

function plot_full_angle_truth_axis(data, axis_index, center_deg)
tracks = selected_truth_tracks(data);
if ~isempty(tracks), tracks = tracks([tracks.scope_dim] == 2); end
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).angle_t), continue; end
    value = tracks(i).angle(axis_index, :);
    if axis_index == 1
        value = azimuth_for_plot(value, center_deg);
        value = unwrap_for_plot(value);
    end
    plot(tracks(i).angle_t, value, 'o-', 'Color', colors(i, :), ...
        'LineWidth', 1.2, 'MarkerSize', 3.5, ...
        'DisplayName', truth_display_name(tracks(i)));
end
end

function plot_full_position_truth(data, line_width)
tracks = selected_truth_tracks(data);
if ~isempty(tracks), tracks = tracks([tracks.scope_dim] == 3); end
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).t), continue; end
    plot3(tracks(i).p(1, :), tracks(i).p(2, :), tracks(i).p(3, :), 'o-', ...
        'Color', colors(i, :), 'LineWidth', line_width, 'MarkerSize', 3.5, ...
        'DisplayName', truth_display_name(tracks(i)));
    annotate_truth_start(gca, tracks(i).p, tracks(i).label, colors(i, :));
end
end

function plot_full_position_truth_axis(data, axis_index)
tracks = selected_truth_tracks(data);
if ~isempty(tracks), tracks = tracks([tracks.scope_dim] == 3); end
colors = lines(max(numel(tracks), 1));
for i = 1:numel(tracks)
    if isempty(tracks(i).t), continue; end
    plot(tracks(i).t, tracks(i).p(axis_index, :), 'o-', ...
        'Color', colors(i, :), 'LineWidth', 1.2, 'MarkerSize', 3.5, ...
        'DisplayName', truth_display_name(tracks(i)));
end
end

function tracks = selected_truth_tracks(data)
tracks = repmat(empty_truth_track(), 1, 0);
for i = 1:numel(data)
    if ~isempty(data(i).truth_tracks)
        tracks = data(i).truth_tracks;
        return;
    end
end
end

function annotate_truth_start(ax, values, label, color)
if isempty(values), return; end
first = find(all(isfinite(values), 1), 1);
if isempty(first), return; end
p = values(:, first);
if size(values, 1) >= 3
    text(ax, p(1), p(2), p(3), sprintf(' 真值 %s', label), ...
        'Color', color, 'FontSize', 8, 'VerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
else
    text(ax, p(1), p(2), sprintf(' 真值 %s', label), ...
        'Color', color, 'FontSize', 8, 'VerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
end
end

function track = empty_truth_track()
track = struct('tid', NaN, 'scope_dim', 0, 'label', '', 't', zeros(1, 0), ...
    'p', zeros(3, 0), 'angle_t', zeros(1, 0), 'angle', zeros(2, 0), ...
    'role', 'touched');
end

function name = truth_display_name(track)
switch track.role
    case 'primary'
        prefix = '主匹配真值';
    case 'all'
        prefix = '全部真值';
    otherwise
        prefix = '触达真值';
end
name = sprintf('%s %s（完整）', prefix, track.label);
end

function tol = time_tolerance(t)
tol = max(1e-9, 32 * eps(max(1, max(abs(t)))));
end

function summary = reference_summary(data)
summary = struct('n_tracks', numel(data), ...
    'n_mapped_tracks', nnz(isfinite([data.truth_key])), ...
    'n_tracks_with_angle', nnz([data.n_angle] > 0), ...
    'n_tracks_with_position', nnz([data.n_position] > 0));
end

function y = unwrap_for_plot(x)
x = x(:).';
y = nan(size(x));
good = isfinite(x);
edges = diff([false, good, false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
for q = 1:numel(starts)
    j = starts(q):stops(q);
    y(j) = rad2deg(unwrap(deg2rad(x(j))));
end
end

function [az, el] = break_wrap(az0, el0)
az = []; el = [];
for k = 1:numel(az0)
    if k > 1 && abs(az0(k) - az0(k - 1)) > 180
        az(end + 1) = NaN; el(end + 1) = NaN; %#ok<AGROW>
    end
    az(end + 1) = az0(k); el(end + 1) = el0(k); %#ok<AGROW>
end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function s = axis_name(ax)
if ax == 1, s = '方位角 (deg)'; else, s = '俯仰角 (deg)'; end
end

function v = get_opt(opts, name, fallback)
if isfield(opts, name) && ~isempty(opts.(name)), v = opts.(name); else, v = fallback; end
end
