function plot_joint_tracking_results(est, events, cfg, frames)
%PLOT_JOINT_TRACKING_RESULTS Plot unified angle and spatial track outputs.

if nargin < 4, frames = []; end

max_track_points = max(1, round(get_cfg(cfg, 'plot_max_points_per_track', 2000)));
max_raw_points = max(1, round(get_cfg(cfg, 'plot_max_raw_scatter_points', 100000)));
az_center = get_cfg(cfg, 'plot_azimuth_center_deg', 0);
[ids, hist, total_counts, valid_counts, output_dims] = ...
    collect_output_history(est, max_track_points);
min_life = get_cfg(cfg, 'joint_plot_min_life', 3);
colors = lines(max(numel(ids), 1));
all2 = output_dims == 2 & valid_counts > 0;
all3 = output_dims == 3 & valid_counts > 0;
has2 = all2 & valid_counts >= min_life;
has3 = all3 & valid_counts >= min_life;
created_figures = gobjects(0, 1);
n_decimated_tracks = nnz(total_counts > max_track_points);
fprintf(['[绘图航迹口径] 二维显示=%d/%d，三维显示=%d/%d；' ...
    '分子要求对应维度有效输出点数>=%d，分母为全部正式公开ID。\n'], ...
    nnz(has2), nnz(all2), nnz(has3), nnz(all3), min_life);
if n_decimated_tracks > 0
    fprintf(['[绘图显示抽样] %d条航迹超过%d点；仅图中等间隔抽样，' ...
        '滤波输出、评价和review结果仍保留全量。\n'], ...
        n_decimated_tracks, max_track_points);
end

% Figure 1: condensed active RAE measurements in tracking-frame XYZ.
% These are the actual active spatial measurements presented to the filter;
% display decimation never changes filtering, evaluation, or saved results.
active_xyz = collect_active_xyz(events);
n_active_xyz = size(active_xyz, 2);
active_xyz = limit_scatter_points(active_xyz, max_raw_points);
created_figures(end + 1) = figure('Name', '主动量测点', ...
    'Position', [50, 100, 900, 750]);
if ~isempty(active_xyz)
    plot3(active_xyz(1, :), active_xyz(2, :), active_xyz(3, :), '.', ...
        'Color', [0.7 0.7 0.7], 'MarkerSize', 4);
end
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
title(sprintf('主动量测点（凝聚后共%d点，图中最多%d点）', ...
    n_active_xyz, max_raw_points));
grid on; box on; axis equal; view(45, 30);

% Filter-event angle observations form the gray 2-D tracking background.
% The combined gray cloud is presentation only and never participates in
% association or evaluation.
[act_rae_ang, act_ae_only, pas_ang] = collect_raw_angles(events);
act_rae_ang(1, :) = azimuth_for_plot(act_rae_ang(1, :), az_center);
act_ae_only(1, :) = azimuth_for_plot(act_ae_only(1, :), az_center);
pas_ang(1, :) = azimuth_for_plot(pas_ang(1, :), az_center);
raw_counts = [size(act_rae_ang, 2), size(act_ae_only, 2), size(pas_ang, 2)];
raw_angle_background_count = sum(raw_counts);
background_per_type = max(1, ceil(max_raw_points / 3));
raw_angle_background = limit_scatter_points( ...
    [limit_scatter_points(act_rae_ang, background_per_type), ...
    limit_scatter_points(act_ae_only, background_per_type), ...
    limit_scatter_points(pas_ang, background_per_type)], max_raw_points);

% Figure 2: gray measurement background plus colored formal 2-D outputs.
% Formal 3-D samples are masked out.
if any(has2)
    created_figures(end + 1) = figure('Name', '二维角度滤波航迹总体图', ...
        'Position', [80, 100, 920, 720]);
    hold on; grid on; box on;
    if ~isempty(raw_angle_background)
        plot(raw_angle_background(1, :), raw_angle_background(2, :), '.', ...
            'Color', [0.78 0.78 0.78], 'MarkerSize', 4, ...
            'HandleVisibility', 'off');
    end
    handles = gobjects(1, 0); labels = cell(1, 0);
    idx2 = find(has2);
    for q = 1:numel(idx2)
        i = idx2(q);
        [az, el] = angle_mode_series(hist(i), 2, az_center);
        h = plot(az, el, '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        handles(end + 1) = h; %#ok<AGROW>
        labels{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
        annotate_track_start(gca, [az; el], ids(i), colors(i, :));
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    xlim(az_center + [-180, 180]);
    title(sprintf(['二维角度量测背景与滤波航迹（原始灰色量测%d点，' ...
        '绘图%d/%d条，至少%d个有效二维输出点）'], ...
        raw_angle_background_count, numel(idx2), nnz(all2), min_life));
    if ~isempty(handles), legend(handles, labels, 'Location', 'bestoutside'); end
    hold off;
end

% Figure 3: post-condensation measurements, split by physical type. Read
% them directly from frames so this diagnostic and the pre-condensation
% figure use the same observed-angle coordinate convention.
post_rae_ang = act_rae_ang;
post_ae_only = act_ae_only;
post_pas_ang = pas_ang;
if ~isempty(frames)
    [post_rae_ang, post_ae_only, post_pas_ang] = ...
        collect_postcondense_angles(frames);
    post_rae_ang(1, :) = azimuth_for_plot(post_rae_ang(1, :), az_center);
    post_ae_only(1, :) = azimuth_for_plot(post_ae_only(1, :), az_center);
    post_pas_ang(1, :) = azimuth_for_plot(post_pas_ang(1, :), az_center);
end
post_counts = [size(post_rae_ang, 2), size(post_ae_only, 2), ...
    size(post_pas_ang, 2)];
post_rae_ang = limit_scatter_points(post_rae_ang, max_raw_points);
post_ae_only = limit_scatter_points(post_ae_only, max_raw_points);
post_pas_ang = limit_scatter_points(post_pas_ang, max_raw_points);
created_figures(end + 1) = figure('Name', '凝聚后二维角度量测', ...
    'Position', [120, 130, 920, 720]);
hold on; grid on; box on;
if ~isempty(post_rae_ang)
    plot(post_rae_ang(1, :), post_rae_ang(2, :), '.', 'Color', [0.15 0.45 0.80], ...
        'MarkerSize', 7, 'DisplayName', '主动RAE中的角度');
end
if ~isempty(post_ae_only)
    plot(post_ae_only(1, :), post_ae_only(2, :), 'x', 'Color', [0.20 0.65 0.35], ...
        'MarkerSize', 6, 'DisplayName', '主动AE-only');
end
if ~isempty(post_pas_ang)
    plot(post_pas_ang(1, :), post_pas_ang(2, :), '.', 'Color', [0.85 0.35 0.15], ...
        'MarkerSize', 7, 'DisplayName', '物理被动AE');
end
xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
xlim(az_center + [-180, 180]);
title(sprintf(['凝聚后二维角度量测（主动RAE角度%d点，主动AE-only%d点，' ...
    '物理被动AE%d点；图中每类最多%d点）'], post_counts(1), ...
    post_counts(2), post_counts(3), max_raw_points));
if ~isempty(post_rae_ang) || ~isempty(post_ae_only) || ~isempty(post_pas_ang)
    legend('Location', 'best');
end
hold off;

% Additional diagnostic: active AE before condensation versus original passive AE.
if ~isempty(frames) && isfield(frames, 'active_ang_precondense')
    [rae_pre, ae_only_pre, pas_pre] = collect_precondense_angles(frames);
    rae_pre(1, :) = azimuth_for_plot(rae_pre(1, :), az_center);
    ae_only_pre(1, :) = azimuth_for_plot(ae_only_pre(1, :), az_center);
    pas_pre(1, :) = azimuth_for_plot(pas_pre(1, :), az_center);
    pre_counts = [size(rae_pre, 2), size(ae_only_pre, 2), size(pas_pre, 2)];
    rae_pre = limit_scatter_points(rae_pre, max_raw_points);
    ae_only_pre = limit_scatter_points(ae_only_pre, max_raw_points);
    pas_pre = limit_scatter_points(pas_pre, max_raw_points);
    created_figures(end + 1) = figure('Name', '凝聚前主动与被动AE量测', ...
        'Position', [140, 145, 920, 720]);
    hold on; grid on; box on;
    if ~isempty(rae_pre)
        plot(rae_pre(1, :), rae_pre(2, :), '.', 'Color', [0.15 0.45 0.80], ...
            'MarkerSize', 7, 'DisplayName', '主动RAE角度（凝聚前）');
    end
    if ~isempty(ae_only_pre)
        plot(ae_only_pre(1, :), ae_only_pre(2, :), 'x', ...
            'Color', [0.20 0.65 0.35], 'MarkerSize', 6, ...
            'DisplayName', '主动AE-only');
    end
    if ~isempty(pas_pre)
        plot(pas_pre(1, :), pas_pre(2, :), '.', 'Color', [0.85 0.35 0.15], ...
            'MarkerSize', 7, 'DisplayName', '被动AE（原始）');
    end
    xlabel('方位角 (deg)'); ylabel('俯仰角 (deg)');
    xlim(az_center + [-180, 180]);
    title(sprintf(['凝聚前AE量测（主动RAE角度%d点，主动AE-only%d点，' ...
        '物理被动AE%d点；图中每类最多%d点）'], pre_counts(1), ...
        pre_counts(2), pre_counts(3), max_raw_points));
    if ~isempty(rae_pre) || ~isempty(ae_only_pre) || ~isempty(pas_pre)
        legend('Location', 'best');
    end
    hold off;
end

% Figure 4: formal 3-D output only. 2-D intervals remain as line breaks.
if any(has3)
    created_figures(end + 1) = figure('Name', '三维主动空间滤波航迹', ...
        'Position', [160, 160, 980, 760]);
    hold on; grid on; box on;
    h3 = gobjects(1, 0); l3 = cell(1, 0);
    idx3 = find(has3);
    for q = 1:numel(idx3)
        i = idx3(q);
        pos = position_mode_series(hist(i), 3);
        h = plot3(pos(1, :), pos(2, :), pos(3, :), ...
            '.-', 'Color', colors(i, :), 'LineWidth', 1.0, 'MarkerSize', 7);
        h3(end + 1) = h; %#ok<AGROW>
        l3{end + 1} = sprintf('Track %d', ids(i)); %#ok<AGROW>
        annotate_track_start(gca, pos, ids(i), colors(i, :));
    end
    xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
    title(sprintf('三维主动空间航迹（绘图%d/%d条，至少%d个有效三维输出点）', ...
        numel(idx3), nnz(all3), min_life));
    view(45, 30); axis equal;
    if ~isempty(h3), legend(h3, l3, 'Location', 'bestoutside'); end
    hold off;
end

created_figures(end + 1) = figure('Name', '逻辑航迹模式统计', ...
    'Position', [200, 190, 900, 420]);
plot(est.filter_times, est.N2, 'LineWidth', 1.3, 'DisplayName', '二维输出'); hold on;
plot(est.filter_times, est.N, 'LineWidth', 1.3, 'DisplayName', '三维输出');
plot(est.filter_times, est.N_total, 'k:', 'LineWidth', 1.1, 'DisplayName', '逻辑航迹总数');
grid on; box on; xlabel('Time (s)'); ylabel('航迹数'); title('逻辑航迹输出维度');
legend('Location', 'best'); hold off;

if isfield(cfg, 'plot_save_dir') && ~isempty(cfg.plot_save_dir)
    save_all_open_figures(cfg.plot_save_dir, created_figures, cfg);
end
end

function [az, el] = angle_mode_series(h, mode, center_deg)
valid = h.dim == mode & isfinite(h.az) & isfinite(h.el);
az = azimuth_for_plot(h.az, center_deg); el = h.el;
az(~valid) = NaN; el(~valid) = NaN;
[az, el] = break_az_wrap(az, el);
end

function pos = position_mode_series(h, mode)
valid = h.dim == mode & all(isfinite(h.pos), 1);
pos = h.pos;
pos(:, ~valid) = NaN;
end

function [ids, hist, counts, valid_counts, dims] = collect_output_history(est, max_points)
template = struct('t', [], 'az', [], 'el', [], 'dim', [], ...
    'pos', zeros(3, 0));
ids = zeros(1, 0); counts = zeros(1, 0); valid_counts = zeros(1, 0);
dims = zeros(1, 0);
if ~isfield(est, 'output') && isfield(est, 'output_history')
    [ids, hist, counts, valid_counts, dims] = joint_review_output( ...
        'history', est.output_history, max_points);
    return;
end
if isfield(est, 'output_id_map') && ~isempty(est.output_id_map)
    ids = reshape([est.output_id_map.output_id], 1, []);
    counts = reshape([est.output_id_map.n_output_points], 1, []);
    dims = reshape([est.output_id_map.output_dim], 1, []);
    [ids, order] = sort(ids);
    counts = counts(order); dims = dims(order);
else
    nonempty = ~cellfun('isempty', est.output);
    if any(nonempty)
        chunks = cellfun(@(out) reshape([out.id], 1, []), ...
            est.output(nonempty), 'UniformOutput', false);
        all_ids = [chunks{:}];
        [ids, ~, group] = unique(all_ids);
        counts = accumarray(group(:), 1, [numel(ids), 1]).';
        dims = zeros(size(ids));
        for k = find(nonempty(:)).'
            out = est.output{k};
            for q = 1:numel(out)
                i = find(ids == out(q).id, 1);
                if dims(i) == 0, dims(i) = out(q).output_dim; end
            end
        end
    end
end
if isempty(ids)
    hist = repmat(template, 0, 1);
    return;
end
if any(~isfinite(ids) | ids <= 0 | ids ~= round(ids)) || ...
        any(~ismember(dims, [2, 3]))
    error('plot_joint_tracking_results:InvalidPublicOutputId', ...
        '绘图输入包含无效公开ID或混乱的输出维度。');
end
slot_by_id = zeros(1, max(ids));
slot_by_id(ids) = 1:numel(ids);
hist = repmat(template, numel(ids), 1);
selected_ordinals = cell(numel(ids), 1);
for i = 1:numel(ids)
    n_show = min(counts(i), max_points);
    if n_show == counts(i)
        selected_ordinals{i} = 1:counts(i);
    else
        selected_ordinals{i} = unique(round(linspace(1, counts(i), n_show)));
    end
    n_show = numel(selected_ordinals{i});
    hist(i).t = nan(1, n_show); hist(i).az = nan(1, n_show);
    hist(i).el = nan(1, n_show); hist(i).dim = zeros(1, n_show);
    hist(i).pos = nan(3, n_show);
end
seen = zeros(1, numel(ids)); stored = zeros(1, numel(ids));
valid_counts = zeros(1, numel(ids));
for k = 1:numel(est.output)
    out = est.output{k};
    for q = 1:numel(out)
        i = slot_by_id(out(q).id);
        seen(i) = seen(i) + 1;
        if out(q).output_dim == 3
            valid_counts(i) = valid_counts(i) + ...
                double(all(isfinite(out(q).position_enu)));
        else
            valid_counts(i) = valid_counts(i) + ...
                double(isfinite(out(q).az_deg) && isfinite(out(q).el_deg));
        end
        next = stored(i) + 1;
        if next > numel(selected_ordinals{i}) || ...
                seen(i) ~= selected_ordinals{i}(next)
            continue;
        end
        stored(i) = next;
        hist(i).t(next) = out(q).t_sec;
        hist(i).az(next) = out(q).az_deg;
        hist(i).el(next) = out(q).el_deg;
        hist(i).dim(next) = out(q).output_dim;
        hist(i).pos(:, next) = out(q).position_enu;
    end
end
if any(seen ~= counts)
    warning('plot_joint_tracking_results:StaleOutputIdMap', ...
        'output_id_map计数与正式输出历史不一致，绘图已按实际读取点数处理。');
end
for i = 1:numel(ids)
    if stored(i) < numel(hist(i).t)
        keep = 1:stored(i);
        hist(i).t = hist(i).t(keep); hist(i).az = hist(i).az(keep);
        hist(i).el = hist(i).el(keep); hist(i).dim = hist(i).dim(keep);
        hist(i).pos = hist(i).pos(:, keep);
    end
end
end

function X = limit_scatter_points(X, max_points)
n = size(X, 2);
if n <= max_points, return; end
keep = unique(round(linspace(1, n, max_points)));
X = X(:, keep);
end

function xyz = collect_active_xyz(events)
n_total = sum(arrayfun(@(e) e.active.n_meas, events));
xyz = nan(3, n_total);
cursor = 0;
for k = 1:numel(events)
    n = events(k).active.n_meas;
    if n <= 0, continue; end
    if ~isfield(events(k).active, 'xyz') || ...
            size(events(k).active.xyz, 1) ~= 3 || ...
            size(events(k).active.xyz, 2) < n
        error('plot_joint_tracking_results:InvalidActiveXYZ', ...
            '事件%d的主动量测数量与XYZ数据不一致。', k);
    end
    xyz(:, cursor + (1:n)) = events(k).active.xyz(:, 1:n);
    cursor = cursor + n;
end
xyz = xyz(:, 1:cursor);
xyz = xyz(:, all(isfinite(xyz), 1));
end

function [active_rae, active_ae_only, passive] = collect_raw_angles(events)
na = sum(arrayfun(@(e) e.active.n_meas, events));
nae = 0; np = 0;
for k = 1:numel(events)
    n = events(k).passive.n_meas;
    if n <= 0, continue; end
    kind = ones(1, n);
    if isfield(events(k).passive, 'kind') && ...
            numel(events(k).passive.kind) == n
        kind = reshape(events(k).passive.kind, 1, []);
    end
    nae = nae + nnz(kind == 2);
    np = np + nnz(kind ~= 2);
end
active_rae = nan(2, na); active_ae_only = nan(2, nae);
passive = nan(2, np);
ia = 0; ie = 0; ip = 0;
for k = 1:numel(events)
    if events(k).active.n_meas > 0
        n = events(k).active.n_meas;
        active_rae(:, ia + (1:n)) = events(k).active.rae(2:3, :);
        ia = ia + n;
    end
    if events(k).passive.n_meas > 0
        n = events(k).passive.n_meas;
        kind = ones(1, n);
        if isfield(events(k).passive, 'kind') && ...
                numel(events(k).passive.kind) == n
            kind = reshape(events(k).passive.kind, 1, []);
        end
        n_ae = nnz(kind == 2); n_passive = nnz(kind ~= 2);
        if n_ae > 0
            active_ae_only(:, ie + (1:n_ae)) = ...
                events(k).passive.ang(:, kind == 2);
            ie = ie + n_ae;
        end
        if n_passive > 0
            passive(:, ip + (1:n_passive)) = ...
                events(k).passive.ang(:, kind ~= 2);
            ip = ip + n_passive;
        end
    end
end
end

function [active_rae, active_ae_only, passive] = collect_precondense_angles(frames)
n_rae = 0; n_ae = 0; np = 0;
for k = 1:numel(frames)
    if isfield(frames(k), 'active_ang_precondense') && ...
            size(frames(k).active_ang_precondense, 1) == 2
        n_all = size(frames(k).active_ang_precondense, 2);
        n_frame_ae = size(field_or_frame( ...
            frames(k), 'active_ae_only', zeros(2, 0)), 2);
        n_ae = n_ae + min(n_frame_ae, n_all);
        n_rae = n_rae + max(0, n_all - n_frame_ae);
    end
    if isfield(frames(k), 'passive_ang') && size(frames(k).passive_ang, 1) == 2
        np = np + size(frames(k).passive_ang, 2);
    end
end

active_rae = nan(2, n_rae);
active_ae_only = nan(2, n_ae);
passive = nan(2, np);
ir = 0; ia = 0; ip = 0;
for k = 1:numel(frames)
    if isfield(frames(k), 'active_ang_precondense') && ...
            size(frames(k).active_ang_precondense, 1) == 2
        A = frames(k).active_ang_precondense;
        n_all = size(A, 2);
        n_frame_ae = min(size(field_or_frame( ...
            frames(k), 'active_ae_only', zeros(2, 0)), 2), n_all);
        n_frame_rae = n_all - n_frame_ae;
        if n_frame_rae > 0
            active_rae(:, ir + (1:n_frame_rae)) = A(:, 1:n_frame_rae);
            ir = ir + n_frame_rae;
        end
        if n_frame_ae > 0
            active_ae_only(:, ia + (1:n_frame_ae)) = ...
                A(:, n_frame_rae + (1:n_frame_ae));
            ia = ia + n_frame_ae;
        end
    end
    if isfield(frames(k), 'passive_ang') && size(frames(k).passive_ang, 1) == 2
        n = size(frames(k).passive_ang, 2);
        passive(:, ip + (1:n)) = frames(k).passive_ang;
        ip = ip + n;
    end
end
active_rae = active_rae(:, all(isfinite(active_rae), 1));
active_ae_only = active_ae_only(:, all(isfinite(active_ae_only), 1));
passive = passive(:, all(isfinite(passive), 1));
end

function [active_rae, active_ae_only, passive] = collect_postcondense_angles(frames)
n_rae = sum(arrayfun(@(f) size(field_or_frame( ...
    f, 'active_rae', zeros(3, 0)), 2), frames));
n_ae = sum(arrayfun(@(f) size(field_or_frame( ...
    f, 'active_ae_only', zeros(2, 0)), 2), frames));
n_pas = sum(arrayfun(@(f) size(field_or_frame( ...
    f, 'passive_ang', zeros(2, 0)), 2), frames));
active_rae = nan(2, n_rae);
active_ae_only = nan(2, n_ae);
passive = nan(2, n_pas);
ir = 0; ia = 0; ip = 0;
for k = 1:numel(frames)
    R = field_or_frame(frames(k), 'active_rae', zeros(3, 0));
    if size(R, 1) >= 3
        n = size(R, 2);
        active_rae(:, ir + (1:n)) = R(2:3, :);
        ir = ir + n;
    end
    A = field_or_frame(frames(k), 'active_ae_only', zeros(2, 0));
    if size(A, 1) >= 2
        n = size(A, 2);
        active_ae_only(:, ia + (1:n)) = A(1:2, :);
        ia = ia + n;
    end
    P = field_or_frame(frames(k), 'passive_ang', zeros(2, 0));
    if size(P, 1) >= 2
        n = size(P, 2);
        passive(:, ip + (1:n)) = P(1:2, :);
        ip = ip + n;
    end
end
active_rae = active_rae(:, 1:ir);
active_ae_only = active_ae_only(:, 1:ia);
passive = passive(:, 1:ip);
active_rae = active_rae(:, all(isfinite(active_rae), 1));
active_ae_only = active_ae_only(:, all(isfinite(active_ae_only), 1));
passive = passive(:, all(isfinite(passive), 1));
end

function value = field_or_frame(frame, name, fallback)
if isfield(frame, name) && ~isempty(frame.(name))
    value = frame.(name);
else
    value = fallback;
end
end

function [az, el] = break_az_wrap(az0, el0)
az = zeros(1, 0); el = zeros(1, 0);
for k = 1:numel(az0)
    if k > 1 && isfinite(az0(k - 1)) && isfinite(az0(k)) && abs(az0(k) - az0(k - 1)) > 180
        az(end + 1) = NaN; el(end + 1) = NaN; %#ok<AGROW>
    end
    az(end + 1) = az0(k); el(end + 1) = el0(k); %#ok<AGROW>
end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end
