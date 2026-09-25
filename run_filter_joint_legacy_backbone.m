function [est, events] = run_filter_joint_legacy_backbone( ...
        fused_xyz, fused_R, frame_times, passive_bearing, platform, ...
        fused_ids, event_meta, cfg, source_events)
%RUN_FILTER_JOINT_LEGACY_BACKBONE Extend the mature 3-D tracker with 2-D tracks.

K = numel(fused_xyz);
if nargin < 9, source_events = []; end
if isempty(source_events), source_events = event_meta; end
if isempty(passive_bearing), passive_bearing = cell(K, 1); end
passive_enabled = logical(get_cfg(cfg, 'passive_bearing_enabled', true));
[~, n_passive_before_selection, n_active_before_selection] = ...
    angle_source_counts(passive_bearing);
[passive_bearing, ~] = select_enabled_angle_sources( ...
    passive_bearing, passive_enabled);

% TXT files are storage shards, not sensors. Optional de-duplication only
% removes exact overlapping rows before all passive shards share one source.
[passive_bearing, shard_stats] = dedup_angle_packets(passive_bearing, cfg);
passive_bearing = normalize_passive_sensor(passive_bearing);
[n_angle, n_passive_angle, n_active_angle] = angle_source_counts(passive_bearing);
n_active_range = sum(cellfun(@(z) size(z, 2), fused_xyz));
has_angle = n_angle > 0;
events = make_filter_events(fused_xyz, fused_R, frame_times, passive_bearing, ...
    fused_ids, platform, cfg, source_events);
backbone_event_meta = event_meta;
if isempty(backbone_event_meta)
    % The mature 3-D API historically assumes active-only events when its
    % metadata argument is omitted.  Preserve that standalone behavior in
    % the backbone, but supply truthful wrapper metadata so a pure-passive
    % call can advance the online mature 2-D confirmation controller.
    backbone_event_meta = infer_backbone_event_meta(events);
end

fprintf(['  联合输入能力: 主动RAE=%d, 独立AE=%d ' ...
    '(被动=%d, 主动AE-only=%d)\n'], ...
    n_active_range, n_angle, n_passive_angle, n_active_angle);

cfg3 = cfg;
cfg3.joint_extension_enabled = has_angle;
cfg3.passive_bearing_enabled = has_angle;
% 单次bearing更新先记为S=3；连续命中由成熟主干折算为S=4，S=4与
% 主动S=1共同参与二维/三维起始确认。这不是“禁止被动辅助确认”。
cfg3.passive_bearing_confirm_hit = false;
cfg3.passive_bearing_update_on_active = true;
cfg3.passive_bearing_update_on_pure = true;
cfg3.passive_bearing_update_active_hit_tracks = true;
cfg3.passive_bearing_min_dt_s = 0;
cfg3.passive_bearing_fast_gate_deg = get_cfg(cfg, ...
    'joint_passive_fast_gate_deg', inf);
cfg3.use_target_id_prior = false;

est3 = run_filter_adapt_ckf(fused_xyz, fused_R, frame_times, cfg3, ...
    passive_bearing, platform, fused_ids, backbone_event_meta);

if has_angle
    if ~isfield(est3, 'joint2d') || isempty(est3.joint2d)
        error('run_filter_joint_legacy_backbone:MissingOnline2D', ...
            'The mature 3-D loop did not return its online angle branch.');
    end
    est2 = est3.joint2d;
    print_online_2d_summary(est2, cfg);
    processing_architecture = 'mature_3d_with_logical_dimension_manager';
else
    est2 = empty_online_2d_estimate(events, cfg);
    fprintf('  纯主动RAE输入: 二维分支与升降维管理未启用。\n');
    processing_architecture = 'mature_3d_only_joint_adapter';
end
residual_map = online_residual_map(est2);
output_offset = round(get_cfg(cfg, 'joint_2d_id_offset', 1000));
offset = round(get_cfg(cfg, 'joint_internal_2d_id_offset', 1000000));
namespace_check_tic = tic;
if maximum_legacy_id(est3) >= offset
    error('run_filter_joint_legacy_backbone:IdNamespaceCollision', ...
        'cfg.joint_internal_2d_id_offset must exceed every possible 3-D track ID.');
end
namespace_check_s = toc(namespace_check_tic);

assembly_tic = tic;
est = assemble_joint_estimate(est3, est2, events, residual_map, ...
    offset, output_offset, platform, cfg);
assembly_s = toc(assembly_tic);
est.timing.namespace_check_s = namespace_check_s;
est.timing.joint_assembly_s = assembly_s;
fprintf(['[联合层后处理耗时] ID上界检查=%.2fs，事件组装及公开ID账本=%.2fs' ...
    '（输出ID首次赋值=%.2fs，确认前关联回填=%.2fs，身份审计=%.2fs，状态日志=%.2fs）。\n'], ...
    namespace_check_s, assembly_s, ...
    field_or(est.timing, 'output_id_assignment_s', NaN), ...
    field_or(est.timing, 'association_id_backfill_s', NaN), ...
    field_or(est.timing, 'output_identity_audit_s', NaN), ...
    field_or(est.timing, 'output_status_log_s', NaN));
est.online_causal = true;
est.processing_passes = 1;
est.processing_architecture = processing_architecture;
est.stats.angle_shard_inputs = shard_stats.n_input;
est.stats.angle_shard_duplicates = shard_stats.n_removed;
est.stats.angle_shard_duplicates_passive = shard_stats.n_removed_passive;
est.stats.angle_shard_duplicates_active = shard_stats.n_removed_active;
est.stats.input_active_rae = n_active_range;
est.stats.input_passive_ae = n_passive_angle;
est.stats.input_active_ae_only = n_active_angle;
est.stats.input_passive_disabled = max(0, n_passive_before_selection - ...
    n_passive_angle - shard_stats.n_removed_passive);
est.stats.input_active_ae_disabled = max(0, n_active_before_selection - ...
    n_active_angle - shard_stats.n_removed_active);
print_joint_contract_summary(est);
end

function [n_total, n_passive, n_active] = angle_source_counts(pb)
n_passive = 0; n_active = 0;
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    n = size(field_or(pb{k}, 'ang_deg', zeros(2, 0)), 2);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    n_passive = n_passive + nnz(kind == 1);
    n_active = n_active + nnz(kind == 2);
end
n_total = n_passive + n_active;
end

function est = empty_online_2d_estimate(events, cfg)
K = numel(events);
empty_cells = cell(K, 1);
est = struct();
for name = {'X', 'P', 'L', 'X2', 'P2', 'L2', 'logical_tracks', ...
        'output', 'tracks', 'assoc', 'companions'}
    est.(name{1}) = empty_cells;
end
est.N = zeros(K, 1); est.N2 = zeros(K, 1); est.N_total = zeros(K, 1);
est.filter_times = reshape([events.t_sec], [], 1);
est.event_meta = events;
est.mode_counts = struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1));
est.timing = struct('total', 0);
est.transition_log = struct('id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
est.stats = struct();
est.confirmation = struct('M', get_cfg(cfg, 'joint_confirm_M', 3), ...
    'N_events', get_cfg(cfg, 'joint_confirm_N', 5), ...
    'passive_group_size', get_cfg(cfg, 'joint_passive_confirm_consecutive_hits', 3), ...
    'passive_max_gap_s', get_cfg(cfg, 'joint_passive_confirm_max_gap_s', 0.2));
est.framework = 'joint_2d3d';
est.output_contract = 'logical_track_v1';
end

function print_online_2d_summary(est2, cfg)
s = field_or(est2, 'stats', struct());
c = field_or(est2, 'confirmation', struct());
fprintf('\n========== 在线二维分支与维度管理 ==========\n');
fprintf('  确认条件: M=%g, 事件窗N=%g, 每组被动命中=%g, 连续间隔<=%.3fs\n', ...
    field_or(c, 'M', NaN), field_or(c, 'N_events', NaN), ...
    field_or(c, 'passive_group_size', NaN), ...
    field_or(c, 'passive_max_gap_s', NaN));
fprintf(['  量测关联=%g, 新生=%g, 新生抑制=%g, 暂态删除=%g, ' ...
    '重复合并=%g\n'], field_or(s, 'passive_assigned', 0), ...
    field_or(s, 'passive_births', 0), ...
    field_or(s, 'passive_birth_suppressed', 0), ...
    field_or(s, 'deleted_tentative', 0), ...
    field_or(s, 'duplicates_merged', 0));
fprintf(['  主动AE到二维候选/验收边=%g/%g, 严格拒绝[NIS=%g, 间断=%g, ' ...
    '方位=%g, 俯仰=%g, LOS=%g], 释放量测=%g\n'], ...
    field_or(s, 'active_2d_candidate_edges', 0), ...
    field_or(s, 'active_2d_accept_edges', 0), ...
    field_or(s, 'active_2d_reject_nis', 0), ...
    field_or(s, 'active_2d_reject_gap', 0), ...
    field_or(s, 'active_2d_reject_az', 0), ...
    field_or(s, 'active_2d_reject_el', 0), ...
    field_or(s, 'active_2d_reject_los', 0), ...
    field_or(s, 'active_2d_released_measurements', 0));
fprintf(['  二维候选/验收边=%g/%g, 严格拒绝[NIS=%g, 间断=%g, ' ...
    '方位=%g, 俯仰=%g, LOS=%g], 释放量测=%g\n'], ...
    field_or(s, 'passive_candidate_edges', 0), ...
    field_or(s, 'passive_accept_edges', 0), ...
    field_or(s, 'passive_reject_nis', 0), ...
    field_or(s, 'passive_reject_gap', 0), ...
    field_or(s, 'passive_reject_az', 0), ...
    field_or(s, 'passive_reject_el', 0), ...
    field_or(s, 'passive_reject_los', 0), ...
    field_or(s, 'passive_released_measurements', 0));
fprintf('  残余独立二维输出: %d点, %d条唯一航迹\n', ...
    sum(est2.N2), numel(output_ids(est2)));
n_quality = field_or(s, 'assoc_quality_observed', 0);
if n_quality > 0
    mean_quality = field_or(s, 'assoc_quality_sum', 0) / n_quality;
else
    mean_quality = NaN;
end
fprintf(['  联合关联可信度[%s]: 观察=%g, 低可信=%g, 若on将拒绝候选边=%g, ' ...
    '平均=%.3f\n'], get_cfg(cfg, 'assoc_quality_mode', 'off'), n_quality, ...
    field_or(s, 'assoc_quality_low', 0), ...
    field_or(s, 'assoc_quality_rejected_edges', 0), mean_quality);
fprintf(['  联合试探质量: 检查=%g, 未达标=%g, 质量超时删除=%g；' ...
    '角度稳健降权=%g(最大R膨胀=%.2f)\n'], ...
    field_or(s, 'track_quality_checks', 0), ...
    field_or(s, 'track_quality_not_ready', 0), ...
    field_or(s, 'track_quality_deleted', 0), ...
    field_or(s, 'bearing_robust_downweighted', 0), ...
    field_or(s, 'bearing_robust_max_inflation', 1));
fprintf(['  纯二维竞争保护: 稳定锁定=%g, 歧义暂缓航迹=%g, ' ...
    '歧义暂缓量测=%g；视线角速度趋势修正=%g次\n'], ...
    field_or(s, 'passive_stable_locked', 0), ...
    field_or(s, 'passive_ambiguity_held_tracks', 0), ...
    field_or(s, 'passive_ambiguity_held_measurements', 0), ...
    field_or(s, 'los_trend_corrections', 0));
log = field_or(est2, 'transition_log', struct('reason', {}));
reasons = {log.reason};
fprintf(['  跨维重复抑制=%g, 内部logical接管=%g ' ...
    '(暂态=%g, 晚生=%g, 已绑定=%g), 质量降二维=%g, 恢复三维=%g\n'], ...
    field_or(s, 'cross_dimension_suppressed', 0), ...
    field_or(s, 'cross_dimension_adopted', 0), ...
    field_or(s, 'cross_dimension_tentative_suppressed', 0), ...
    field_or(s, 'cross_dimension_late_suppressed', 0), ...
    field_or(s, 'cross_dimension_bound_suppressed', 0), ...
    nnz(strcmp(reasons, '3d_quality_degraded')), ...
    nnz(strcmp(reasons, '2d_to_3d_2of3_confirmed')));
fprintf('  外部三维ID重绑定: 候选=%g, 提交=%g, 歧义拒绝=%g, detached被动更新=%g\n', ...
    field_or(s, 'external_rebind_candidates', 0), ...
    field_or(s, 'external_rebind_committed', 0), ...
    field_or(s, 'external_rebind_ambiguous', 0), ...
    field_or(s, 'detached_passive_assigned', 0));
fprintf(['    漏斗: 活动新ID=%g, 旧companion=%g, 投影无效=%g, ' ...
    '拒绝[角度=%g,角速度=%g,NIS=%g], 分配=%g, ' ...
    '等待[历史=%g,三维就绪=%g,旧ID退出=%g]\n'], ...
    field_or(s, 'external_rebind_active_unknown', 0), ...
    field_or(s, 'external_rebind_eligible', 0), ...
    field_or(s, 'external_rebind_projection_invalid', 0), ...
    field_or(s, 'external_rebind_reject_angle', 0), ...
    field_or(s, 'external_rebind_reject_rate', 0), ...
    field_or(s, 'external_rebind_reject_switch', 0), ...
    field_or(s, 'external_rebind_assigned', 0), ...
    field_or(s, 'external_rebind_wait_history', 0), ...
    field_or(s, 'external_rebind_wait_ready', 0), ...
    field_or(s, 'external_rebind_wait_bound', 0));
end

function print_joint_contract_summary(est)
s = est.stats;
fprintf('\n[架构]\n');
fprintf('  3D backbone = run_filter_adapt_ckf\n');
fprintf('  2D backbone = mature passive IMM-KF in run_filter_joint_2d3d\n');
fprintf('  manager     = run_filter_joint_legacy_backbone\n');
fprintf('[主动RAE去向]\n');
fprintf(['  输入=%g, 关联成熟3D=%g, 新生=%g, 明确抑制=%g, ' ...
    '未解释=%g, 正式3D输出点=%g\n'], ...
    field_or(s, 'active_measurements', 0), ...
    field_or(s, 'active_assigned', 0), field_or(s, 'active_births', 0), ...
    field_or(s, 'active_birth_suppressed', 0), ...
    field_or(s, 'active_unaccounted', 0), sum(est.N));
fprintf('[被动AE去向]\n');
fprintf(['  输入=%g, ->3D branch=%g, ->2D branch=%g, 未送入支路=%g, ' ...
    '明确抑制=%g, 未解释=%g, 正式2D输出点=%g\n'], ...
    field_or(s, 'passive_measurements', 0), ...
    field_or(s, 'passive_to_3d', 0), field_or(s, 'passive_to_2d', 0), ...
    field_or(s, 'passive_not_routed', 0), ...
    field_or(s, 'passive_birth_suppressed', 0), ...
    field_or(s, 'passive_unaccounted', 0), sum(est.N2));
fprintf('[切换]\n');
fprintf(['  pending 3D->2D事件记录=%g, committed 3D->2D=%g, ' ...
    'blocked: 2D not ready=%g\n'], ...
    field_or(s, 'pending_3d_to_2d_event_records', 0), ...
    field_or(s, 'switch_3d_to_2d_committed', 0), ...
    field_or(s, 'switch_3d_to_2d_blocked_not_ready', 0));
fprintf(['  pending 2D->3D事件记录=%g, committed 2D->3D=%g, ' ...
    '3D shadow/rebind候选输出=%g\n'], ...
    field_or(s, 'pending_2d_to_3d_event_records', 0), ...
    field_or(s, 'switch_2d_to_3d_committed', 0), ...
    field_or(s, 'shadow_3d_candidates', 0));
fprintf('  同一事件重复内部logical输出抑制=%g\n', ...
    field_or(s, 'duplicate_outputs_suppressed', 0));
if isfield(est, 'output_id_map') && ~isempty(est.output_id_map)
    map = est.output_id_map;
    ids3 = [map([map.output_dim] == 3).output_id];
    ids2 = [map([map.output_dim] == 2).output_id];
    fprintf('[正式输出ID]\n');
    fprintf('  三维=%s；二维=%s；内部logical ID仅保留用于切换事务。\n', ...
        compact_id_range(ids3), compact_id_range(ids2));
end
if isfield(est, 'output_identity_audit') && ~isempty(est.output_identity_audit)
    a = est.output_identity_audit;
    fprintf('[三维输出身份对账]\n');
    fprintf(['  成熟主干external ID=%d；联合三维branch ID=%d；' ...
        '公开三维ID=%d；绘图门限(>=%d点)后=%d。\n'], ...
        a.n_mature_3d_external_ids, a.n_joint_3d_branch_ids, ...
        a.n_public_3d_ids, a.plot_min_life, a.n_plot_visible_3d_ids);
    fprintf(['  manager提前三维输出但未进入成熟主干正式集=%d；' ...
        '成熟主干有输出但联合层无三维正式输出=%d；' ...
        '已折叠内部owner别名=%d。\n'], ...
        a.n_manager_only_3d_branches, a.n_mature_without_joint_3d_output, ...
        a.n_internal_owner_aliases_collapsed);
    if ~isempty(a.manager_only_3d_branch_ids)
        fprintf('  manager-only external branch: %s\n', ...
            mat2str(a.manager_only_3d_branch_ids));
    end
    if ~isempty(a.mature_without_joint_3d_ids)
        fprintf('  mature-only external branch: %s\n', ...
            mat2str(a.mature_without_joint_3d_ids));
    end
    if ~isempty(a.short_3d_public_ids)
        fprintf('  少于%d个有效三维输出点、未绘制的公开ID: %s\n', ...
            a.plot_min_life, mat2str(a.short_3d_public_ids));
    end
end
end

function text = compact_id_range(ids)
ids = unique(ids);
if isempty(ids)
    text = '无';
elseif numel(ids) == 1
    text = sprintf('%g', ids);
else
    text = sprintf('%g..%g（%d条）', ids(1), ids(end), numel(ids));
end
end

function [pb, has_active_angle] = select_enabled_angle_sources(pb, passive_enabled)
has_active_angle = false;
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    ang = field_or(pb{k}, 'ang_deg', zeros(2, 0)); n = size(ang, 2);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    has_active_angle = has_active_angle || any(kind == 2);
    keep = kind == 2 | (passive_enabled & kind == 1);
    if all(keep), continue; end
    R = normalize_covariance(field_or(pb{k}, 'R_deg2', []), 2, n, eye(2));
    pb{k}.ang_deg = ang(:, keep); pb{k}.R_deg2 = R(:, :, keep);
    pb{k}.src = sized_row(field_or(pb{k}, 'src', []), n, NaN); pb{k}.src = pb{k}.src(keep);
    pb{k}.shard = sized_row(field_or(pb{k}, 'shard', []), n, NaN); pb{k}.shard = pb{k}.shard(keep);
    pb{k}.kind = kind(keep);
    ids = sized_row(field_or(pb{k}, 'tracklet_id', []), n, NaN);
    pb{k}.tracklet_id = ids(keep);
    tt = sized_row(field_or(pb{k}, 't_sec', []), n, NaN); pb{k}.t_sec = tt(keep);
    pb{k}.n_meas = nnz(keep);
end
end

function pb = normalize_passive_sensor(pb)
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    n = field_or(pb{k}, 'n_meas', size(field_or(pb{k}, 'ang_deg', zeros(2, 0)), 2));
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    normalized = ones(1, n);
    normalized(kind == 2) = 2;
    pb{k}.kind = kind;
    pb{k}.src = normalized;
end
end

function [pb, stats] = dedup_angle_packets(pb, cfg)
stats = struct('n_input', 0, 'n_removed', 0, ...
    'n_removed_passive', 0, 'n_removed_active', 0);
for k = 1:numel(pb)
    if ~isempty(pb{k}) && isstruct(pb{k})
        stats.n_input = stats.n_input + size(field_or( ...
            pb{k}, 'ang_deg', zeros(2, 0)), 2);
    end
end
if ~get_cfg(cfg, 'joint_shard_dedup_enabled', false), return; end
time_gate = get_cfg(cfg, 'joint_shard_duplicate_time_s', 1e-9);
angle_gate = get_cfg(cfg, 'joint_shard_duplicate_angle_deg', 1e-9);
for k = 1:numel(pb)
    if isempty(pb{k}) || ~isstruct(pb{k}), continue; end
    ang = field_or(pb{k}, 'ang_deg', zeros(2, 0)); n = size(ang, 2);
    if n < 2, continue; end
    src = sized_row(field_or(pb{k}, 'src', nan(1, n)), n, NaN);
    shard = sized_row(field_or(pb{k}, 'shard', src), n, NaN);
    kind = sized_row(field_or(pb{k}, 'kind', ones(1, n)), n, 1);
    tt = sized_row(field_or(pb{k}, 't_sec', []), n, NaN);
    keep = zeros(1, 0);
    for q = 1:n
        duplicate = false;
        for r = reshape(keep, 1, [])
            different_shard = isfinite(shard(q)) && isfinite(shard(r)) && ...
                shard(q) ~= shard(r);
            same_time = isfinite(tt(q)) && isfinite(tt(r)) && ...
                abs(tt(q) - tt(r)) <= time_gate;
            same_angle = abs(angle_diff(ang(1, q), ang(1, r))) <= angle_gate && ...
                abs(ang(2, q) - ang(2, r)) <= angle_gate;
            if different_shard && kind(q) == kind(r) && same_time && same_angle
                duplicate = true;
                break;
            end
        end
        if ~duplicate, keep(end + 1) = q; end %#ok<AGROW>
    end
    R = normalize_covariance(field_or(pb{k}, 'R_deg2', []), 2, n, eye(2));
    ids = sized_row(field_or(pb{k}, 'tracklet_id', []), n, NaN);
    removed = true(1, n); removed(keep) = false;
    stats.n_removed = stats.n_removed + nnz(removed);
    stats.n_removed_passive = stats.n_removed_passive + nnz(removed & kind == 1);
    stats.n_removed_active = stats.n_removed_active + nnz(removed & kind == 2);
    pb{k}.ang_deg = ang(:, keep); pb{k}.R_deg2 = R(:, :, keep);
    pb{k}.src = src(keep); pb{k}.kind = kind(keep);
    pb{k}.shard = shard(keep);
    pb{k}.tracklet_id = ids(keep); pb{k}.t_sec = tt(keep);
    pb{k}.n_meas = numel(keep);
end
end

function events = make_filter_events(xyz, R, times, pb, ids, platform, cfg, source)
K = numel(xyz);
events = repmat(event_template(), K, 1);
for k = 1:K
    e = event_template();
    e.cycle_id = k; e.t_sec = times(k); e.t_start = times(k); e.t_end = times(k);
    z = xyz{k}; n = size(z, 2);
    e.has_active = n > 0; e.active.n_meas = n;
    e.active.t_sec = repmat(times(k), 1, n); e.active.xyz = z;
    e.active.R_xyz = normalize_covariance(R{k}, 3, n, diag([2500, 2500, 2500].^2));
    e.active.R_ae = repmat(diag([get_cfg(cfg, 'sigma_az_deg', 0.08)^2, ...
        get_cfg(cfg, 'sigma_el_deg', 0.06)^2]), 1, 1, n);
    e.active.has_range = true(1, n); e.active.src = ones(1, n);
    e.active.ids = sized_row(cell_value(ids, k, []), n, NaN);
    e.active.rae = nan(3, n);
    sensor = platform_enu(times(k), platform, cfg);
    for j = 1:n
        rel = z(:, j) - sensor;
        e.active.rae(:, j) = [norm(rel); atan2d(rel(1), rel(2)); ...
            atan2d(rel(3), hypot(rel(1), rel(2)))];
    end
    if k <= numel(source)
        source_rae = field_or(source(k), 'active_rae', zeros(3, 0));
        if size(source_rae, 1) >= 3 && size(source_rae, 2) >= n
            source_rae = source_rae(1:3, 1:n);
            good_rae = all(isfinite(source_rae), 1);
            e.active.rae(:, good_rae) = source_rae(:, good_rae);
        end
        source_t = sized_row(field_or(source(k), 'active_t', []), n, NaN);
        good_t = isfinite(source_t);
        e.active.t_sec(good_t) = source_t(good_t);
        if any(good_t)
            e.t_start = min([e.t_start, source_t(good_t)]);
            e.t_end = max([e.t_end, source_t(good_t)]);
        end
    end

    p = cell_value(pb, k, []);
    if ~isempty(p) && isstruct(p)
        ang = field_or(p, 'ang_deg', zeros(2, 0)); np = size(ang, 2);
        e.has_passive = np > 0; e.passive.n_meas = np;
        e.passive.ang = ang;
        e.passive.R_ae = normalize_covariance(field_or(p, 'R_deg2', []), 2, np, ...
            diag([get_cfg(cfg, 'sigma_passive_az_deg', 0.05)^2, ...
            get_cfg(cfg, 'sigma_passive_el_deg', 0.04)^2]));
        e.passive.ids = sized_row(field_or(p, 'tracklet_id', []), np, NaN);
        e.passive.src = sized_row(field_or(p, 'src', ones(1, np)), np, 1);
        e.passive.kind = sized_row(field_or(p, 'kind', ones(1, np)), np, 1);
        pt = field_or(p, 't_sec', times(k));
        e.passive.t_sec = sized_row(pt, np, times(k));
        if np > 0
            e.t_start = min([e.t_start, e.passive.t_sec]);
            e.t_end = max([e.t_end, e.passive.t_sec]);
        end
    end
    if k <= numel(source) && isfield(source(k), 't_start')
        e.t_start = min(e.t_start, source(k).t_start);
        e.t_end = max(e.t_end, source(k).t_end);
    end
    events(k) = e;
end
end

function meta = infer_backbone_event_meta(events)
K = numel(events);
meta = repmat(struct('has_active', false, 'has_passive', false, ...
    'miss_cycle', false, 'confirm_cycle', false, ...
    'n_active', 0, 'n_passive', 0, 't_start', NaN, 't_end', NaN), K, 1);
for k = 1:K
    meta(k).has_active = logical(events(k).has_active);
    meta(k).has_passive = logical(events(k).has_passive);
    meta(k).miss_cycle = meta(k).has_active;
    meta(k).confirm_cycle = meta(k).has_active;
    meta(k).n_active = events(k).active.n_meas;
    meta(k).n_passive = events(k).passive.n_meas;
    meta(k).t_start = events(k).t_start;
    meta(k).t_end = events(k).t_end;
end
end

function map = online_residual_map(est2)
K = numel(est2.event_meta); map = cell(K, 1);
for k = 1:K
    e = est2.event_meta(k);
    if isfield(e, 'passive') && isfield(e.passive, 'original_index')
        map{k} = reshape(e.passive.original_index, 1, []);
    else
        map{k} = 1:e.passive.n_meas;
    end
end
end

function est = assemble_joint_estimate(e3, e2, events, residual_map, ...
        offset, output_offset, platform, cfg)
K = numel(events); est = init_joint_estimate(K, events);
retain_full_history = strcmpi(get_cfg(cfg, 'result_save_mode', 'review'), 'full');
id_registry = init_dimension_output_registry(output_offset);
output_id_assignment_s = 0;
duplicate_outputs_suppressed = 0;
shadow_candidate_outputs = 0;
mature3d = e3;
if isfield(mature3d, 'joint2d'), mature3d = rmfield(mature3d, 'joint2d'); end
if retain_full_history
    est.mature3d = mature3d;
    est.passive2d = e2;
end
for k = 1:K
    out = repmat(output_template(), 0, 1);
    shadow_out = repmat(output_template(), 0, 1);
    sensor = platform_enu(events(k).t_sec, platform, cfg);
    companions = decorate_companion_transactions(companions_at(e2, k), offset);
    est.switch_transactions{k} = companions;
    active_ids = legacy_assoc_ids(e3, k);
    passive_ids = passive_update_ids(e3, k);
    if k <= numel(e3.X) && ~isempty(e3.X{k})
        labels = e3.L{k};
        external_ids = reshape(labels(:, 2), 1, []);
        [pending_index, companion_index, mapped_ids] = ...
            external_branch_lookup(external_ids, companions, offset);
        mature_last_updates = external_track_last_updates(e3, k, external_ids);
        active_hit = ismember(external_ids, active_ids);
        passive_hit = ismember(external_ids, passive_ids);
        for j = 1:size(e3.X{k}, 2)
            external_id = external_ids(j);
            if pending_index(j) > 0
                pending_owner = companions(pending_index(j));
                % The replacement 3-D branch is observable for diagnostics
                % but is not formal before the mapping transaction commits.
                birth_event = labels(j, 1);
                shadow = make_3d_output(external_id, birth_event, ...
                    events(k).t_sec, e3.X{k}(:, j), e3.P{k}(:, :, j), ...
                    external_id, active_hit(j), passive_hit(j), [], sensor, ...
                    mature_last_updates(j));
                shadow.formal = false;
                shadow.mode = 'shadow_3d_rebind_candidate';
                shadow.switch_state = pending_owner.switch_state;
                shadow.shadow_for_logical_id = pending_owner.logical_id;
                shadow_out(end + 1, 1) = shadow; %#ok<AGROW>
                shadow_candidate_outputs = shadow_candidate_outputs + 1;
                continue;
            end
            companion = repmat(companion_template(), 0, 1);
            if companion_index(j) > 0
                companion = companions(companion_index(j));
            end
            if ~isempty(companion) && companion.active_output_dim ~= 3
                continue;
            end
            id = mapped_ids(j);
            birth_event = logical_birth_event(labels(j, 1), companion);
            o = make_3d_output(id, birth_event, events(k).t_sec, ...
                e3.X{k}(:, j), e3.P{k}(:, :, j), external_id, ...
                active_hit(j), passive_hit(j), companion, sensor, ...
                mature_last_updates(j));
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end

    % A confirmed 2-D logical track may upgrade as soon as its attached
    % mature-3D candidate has 2/3 active evidence, before the candidate
    % reaches the mature tracker's stricter publication threshold.
    for j = 1:numel(companions)
        q = companions(j);
        id = logical_id_for_external(q.external_3d_id, q, offset);
        if q.active_output_dim == 3 && q.output_dim == 3 && ...
                ~any([out.id] == id)
            [x, P, birth_event, found] = external_track_state(e3, k, q.external_3d_id);
            if found
                birth_event = logical_birth_event(birth_event, q);
                mature_last_update = external_track_last_update( ...
                    e3, k, q.external_3d_id);
                o = make_3d_output(id, birth_event, events(k).t_sec, x, P, ...
                    q.external_3d_id, ismember(q.external_3d_id, active_ids), ...
                    ismember(q.external_3d_id, passive_ids), q, sensor, ...
                    mature_last_update);
                out(end + 1, 1) = o; %#ok<AGROW>
            end
        elseif q.active_output_dim == 2 && q.output_dim == 2 && ...
                ~any([out.id] == id)
            o = make_companion_2d_output(q, id, events(k).t_sec);
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end

    if k <= numel(e2.output)
        for j = 1:numel(e2.output{k})
            q = e2.output{k}(j);
            if q.output_dim ~= 2, continue; end
            id = q.id + offset;
            if any([out.id] == id), continue; end
            o = output_template(); o.id = id; o.birth_event = q.birth_event;
            o.t_sec = q.t_sec; o.output_dim = 2; o.confirmed = q.confirmed;
            if isfield(q, 'last_update_t'), o.last_update_t = q.last_update_t; end
            o.mode = '2d_angle_only'; o.status_code = 1; o.truth_id = q.truth_id;
            o.az_deg = q.az_deg; o.el_deg = q.el_deg;
            o.angle_state = q.angle_state; o.angle_cov = q.angle_cov;
            o.logical_id = id; o.branch_2d_id = q.id;
            o.active_output_dim = 2; o.switch_state = 'stable_2d';
            out(end + 1, 1) = o; %#ok<AGROW>
        end
    end
    [out, n_duplicate] = arbitrate_logical_outputs(out);
    duplicate_outputs_suppressed = duplicate_outputs_suppressed + n_duplicate;
    validate_transaction_event(out, companions, k);
    id_tic = tic;
    [out, id_registry] = assign_dimension_output_ids( ...
        out, id_registry, k, events(k).t_sec);
    output_id_assignment_s = output_id_assignment_s + toc(id_tic);
    if ~retain_full_history
        out = compact_runtime_outputs(out);
    end
    est.output{k} = out;
    if retain_full_history, est.shadow_output{k} = shadow_out; end
    est = store_joint_event(est, e3, e2, events, residual_map, ...
        k, out, offset, companions, retain_full_history);
end
manager_transition_log = map_manager_transition_log( ...
    field_or(e2, 'transition_log', struct([])), est.switch_transactions, ...
    est.filter_times, offset);
output_id_mapping_tic = tic;
output_id_map = id_registry.records;
est = remap_joint_association_ids(est, output_id_map, output_offset);
validate_dimension_output_ids(est, output_offset);
association_id_backfill_s = toc(output_id_mapping_tic);
output_id_mapping_s = output_id_assignment_s + association_id_backfill_s;
est.output_id_map = output_id_map;
output_identity_audit_tic = tic;
est.output_identity_audit = build_output_identity_audit( ...
    est, e3, output_id_map, cfg);
output_identity_audit_s = toc(output_identity_audit_tic);
est.internal_transition_log = manager_transition_log;
output_status_log_tic = tic;
est.output_status_log = build_transition_log(est);
est.transition_log = map_output_transition_log(manager_transition_log, output_id_map);
est.dimension_transition_log = build_track_dimension_transition_log( ...
    manager_transition_log, est.switch_transactions, est.filter_times, ...
    output_id_map, cfg);
output_status_log_s = toc(output_status_log_tic);
est.framework = 'joint_2d3d';
est.backbone = 'mature_active3d';
est.output_contract = 'dimension_branch_scoped_track_v3';
if retain_full_history
    est.history_level = 'full';
else
    est.history_level = 'review';
end
est.status_legend = struct('angle_only_2d', 1, 'passive_2d', 1, 'active_3d', 2, ...
    'active_passive_3d', 3, 'passive_maintained_3d', 4, 'coast_3d', 5);
est.status_legend.angle_2d_with_3d_shadow = 6;
est.timing.total = field_or(e3.timing, 'total', 0) + field_or(e2.timing, 'total', 0);
est.timing.output_id_mapping_s = output_id_mapping_s;
est.timing.output_id_assignment_s = output_id_assignment_s;
est.timing.association_id_backfill_s = association_id_backfill_s;
est.timing.output_identity_audit_s = output_identity_audit_s;
est.timing.output_status_log_s = output_status_log_s;
est.stats = struct('active_births', field_or(e3.birth_stats, 'n_total', 0), ...
    'passive_births', field_or(e2.stats, 'passive_births', 0), ...
    'passive_equivalent_hits', field_or(e3.passive_bearing_stats, ...
        'n_equivalent_confirm_hits', 0), ...
    'cross_dimension_merges', field_or(e2.stats, 'cross_dimension_adopted', 0), ...
    'cross_dimension_suppressed', field_or(e2.stats, 'cross_dimension_suppressed', 0), ...
    'passive_duplicate_merges', field_or(e2.stats, 'duplicates_merged', 0), ...
    'external_rebinds', field_or(e2.stats, 'external_rebind_committed', 0), ...
    'switch_3d_to_2d_committed', field_or(e2.stats, ...
        'switch_3d_to_2d_committed', 0), ...
    'switch_2d_to_3d_committed', field_or(e2.stats, ...
        'switch_2d_to_3d_committed', 0), ...
    'switch_3d_to_2d_blocked_not_ready', field_or(e2.stats, ...
        'switch_3d_to_2d_blocked_not_ready', 0), ...
    'shadow_3d_candidates', shadow_candidate_outputs, ...
    'duplicate_outputs_suppressed', duplicate_outputs_suppressed);
switch_stats = summarize_switch_transactions(est.switch_transactions);
switch_names = fieldnames(switch_stats);
for switch_index = 1:numel(switch_names)
    est.stats.(switch_names{switch_index}) = switch_stats.(switch_names{switch_index});
end
ledger = summarize_measurement_disposition(est.measurement_disposition);
ledger_names = fieldnames(ledger);
for ledger_index = 1:numel(ledger_names)
    est.stats.(ledger_names{ledger_index}) = ledger.(ledger_names{ledger_index});
end
est.output_freshness = struct( ...
    'three_d', field_or(e3, 'output_freshness', struct()), ...
    'two_d', struct('max_silence_s', get_cfg(cfg, ...
        'joint_2d_output_max_silence_s', 0.5), ...
        'n_suppressed', field_or(e2.stats, 'output_stale_suppressed', 0)));
if ~retain_full_history
    % 正常运行只保留评价、绘图和单轨复查所需账本。成熟滤波器本身已在
    % e3/e2 中完成计算，无需再把协方差、内部航迹和伴随快照复制到联合结果。
    est.switch_transactions = cell(K, 1);
end
end

function o = make_3d_output(id, birth_event, t, x, P, external_id, ...
        active_hit, passive_hit, companion, sensor, mature_last_update)
o = output_template();
o.id = id; o.birth_event = birth_event; o.t_sec = t;
o.output_dim = 3; o.confirmed = true; o.state3d = x; o.cov3d = P;
o.logical_id = id; o.branch_3d_id = external_id; o.active_output_dim = 3;
o.switch_state = 'stable_3d';
o.position_enu = x([1, 4, 7]); o.velocity_enu = x([2, 5, 8]);
o.acceleration_enu = x([3, 6, 9]);
o.last_update_t = mature_last_update;
rel = o.position_enu - sensor;
o.az_deg = atan2d(rel(1), rel(2));
o.el_deg = atan2d(rel(3), hypot(rel(1), rel(2))); o.range_m = norm(rel);
if active_hit && passive_hit
    o.status_code = 3; o.mode = '3d_active_passive';
elseif active_hit
    o.status_code = 2; o.mode = '3d_active';
elseif passive_hit
    o.status_code = 4; o.mode = '3d_passive_maintained';
else
    o.status_code = 5; o.mode = '3d_coast';
end
if ~isempty(companion)
    o.last_update_t = companion.last_update_t;
    o.quality = companion.quality;
    o.mode = companion.mode;
    o.branch_2d_id = companion.branch_2d_id;
    o.switch_state = companion.switch_state;
elseif active_hit || passive_hit
    o.last_update_t = t;
end
end

function [pending_index, companion_index, logical_ids] = ...
        external_branch_lookup(external_ids, companions, offset)
pending_index = lookup_companion_indices( ...
    external_ids, companions, 'pending_external_3d_id');
companion_index = lookup_companion_indices( ...
    external_ids, companions, 'external_3d_id');
logical_ids = external_ids;
for q = find(companion_index > 0)
    c = companions(companion_index(q));
    if c.from_2d && isfinite(c.local_2d_id)
        logical_ids(q) = c.local_2d_id + offset;
    elseif isfinite(c.logical_id) && c.logical_id > 0
        logical_ids(q) = c.logical_id;
    end
end
end

function indices = lookup_companion_indices(query, companions, field_name)
indices = zeros(size(query));
if isempty(query) || isempty(companions) || ~isfield(companions, field_name)
    return;
end
values = reshape([companions.(field_name)], 1, []);
valid = isfinite(values);
if ~any(valid), return; end
source_indices = find(valid);
[unique_values, first_local] = unique(values(valid), 'stable');
first_indices = source_indices(first_local);
[found, location] = ismember(query, unique_values);
indices(found) = first_indices(location(found));
end

function values = external_track_last_updates(e3, k, external_ids)
values = nan(size(external_ids));
if isempty(external_ids) || k > numel(e3.tracks) || isempty(e3.tracks{k})
    return;
end
s = e3.tracks{k};
if ~isfield(s, 'L') || isempty(s.L) || ~isfield(s, 'last_update_t')
    return;
end
[found, location] = ismember(external_ids, s.L(2, :));
valid = found & location <= numel(s.last_update_t);
values(valid) = s.last_update_t(location(valid));
end

function t = external_track_last_update(e3, k, external_id)
t = NaN;
if k > numel(e3.tracks) || isempty(e3.tracks{k}), return; end
s = e3.tracks{k};
if ~isfield(s, 'L') || isempty(s.L) || ~isfield(s, 'last_update_t'), return; end
i = find(s.L(2, :) == external_id, 1);
if ~isempty(i) && numel(s.last_update_t) >= i
    t = s.last_update_t(i);
end
end

function o = make_companion_2d_output(q, id, t)
o = output_template();
o.id = id; o.birth_event = q.birth_event; o.t_sec = t;
o.output_dim = 2; o.confirmed = q.confirmed; o.status_code = 6;
o.logical_id = id; o.branch_3d_id = q.branch_3d_id;
o.branch_2d_id = q.branch_2d_id; o.active_output_dim = 2;
o.switch_state = q.switch_state;
o.last_update_t = q.last_update_t;
o.mode = q.mode; o.angle_state = q.angle_state; o.angle_cov = q.angle_cov;
o.az_deg = q.angle_state(1); o.el_deg = q.angle_state(4); o.quality = q.quality;
end

function est = store_joint_event(est, e3, e2, events, residual_map, ...
        k, out, offset, companions, retain_full_history)
idx2 = find([out.output_dim] == 2); idx3 = find([out.output_dim] == 3);
est.N2(k) = numel(idx2); est.N(k) = numel(idx3); est.N_total(k) = numel(out);
if isempty(idx2), est.L2{k} = zeros(0, 2);
else, est.L2{k} = [[out(idx2).birth_event].', [out(idx2).id].']; end
if isempty(idx3), est.L{k} = [];
else, est.L{k} = [[out(idx3).birth_event].', [out(idx3).id].']; end
if retain_full_history
    if isempty(idx2)
        est.X2{k} = zeros(6, 0); est.P2{k} = zeros(6, 6, 0);
    else
        est.X2{k} = cat(2, out(idx2).angle_state);
        est.P2{k} = cat(3, out(idx2).angle_cov);
    end
    if isempty(idx3)
        est.X{k} = []; est.P{k} = [];
    else
        est.X{k} = cat(2, out(idx3).state3d);
        est.P{k} = cat(3, out(idx3).cov3d);
    end
end
est.mode_counts.n2d(k) = est.N2(k); est.mode_counts.n3d(k) = est.N(k);
if isempty(out)
    est.mode_counts.nhold(k) = 0;
else
    est.mode_counts.nhold(k) = nnz(strcmp({out.mode}, 'hold'));
end
if retain_full_history
    est.logical_tracks{k} = outputs_to_snapshots(out);
    if k <= numel(e3.tracks), est.tracks{k} = e3.tracks{k}; end
    est.companions{k} = companions;
end
    est.assoc{k} = combine_assoc(e3, e2, events, residual_map, k, ...
        offset, companions);
    active_reason = repmat({'suppressed_unassociated'}, 1, events(k).active.n_meas);
    passive_reason = repmat({'suppressed_unassociated'}, 1, events(k).passive.n_meas);
    est.measurement_disposition{k} = build_measurement_disposition( ...
        est.assoc{k}, events(k), k, active_reason, passive_reason);
end

function a = combine_assoc(e3, e2, events, residual_map, k, offset, companions)
a = assoc_template();
if k <= numel(e3.assoc) && ~isempty(e3.assoc{k})
    x = e3.assoc{k}; n = numel(x.id);
    mapped_ids = committed_logical_ids(x.id, companions, offset);
    for q = 1:n
        mi = indexed(x, 'meas_index', q, q);
        if mi < 1 || mi > events(k).active.n_meas, continue; end
        id = mapped_ids(q);
        branch_2d_id = paired_2d_branch_for_external(x.id(q), companions);
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(x, q);
        assoc_type = 'active';
        if isfield(x, 'type') && iscell(x.type) && q <= numel(x.type) && ...
                ischar(x.type{q}) && strcmp(x.type{q}, 'active_birth')
            assoc_type = 'active_birth';
        end
        a = append_assoc(a, id, assoc_type, mi, indexed(x, 'tid', q, NaN), ...
            events(k).active.rae(2:3, mi), events(k).active.xyz(:, mi), ...
            NaN, 3, innovation, nis, group_size, innovation_kind, 3, ...
            x.id(q), branch_2d_id);
    end
end
if k <= numel(e3.passive_assoc) && ~isempty(e3.passive_assoc{k})
    d = e3.passive_assoc{k}; jj = find(d.used_mask & isfinite(d.track_id));
    mapped_ids = committed_logical_ids(d.track_id(jj), companions, offset);
    for r = 1:numel(jj)
        mi = jj(r);
        id = mapped_ids(r);
        branch_2d_id = paired_2d_branch_for_external( ...
            d.track_id(jj(r)), companions);
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(d, mi);
        a = append_assoc(a, id, 'passive', mi, ...
            events(k).passive.ids(mi), events(k).passive.ang(:, mi), ...
            nan(3, 1), NaN, 3, innovation, nis, group_size, innovation_kind, 3, ...
            d.track_id(jj(r)), branch_2d_id);
    end
end
if k <= numel(e2.assoc) && ~isempty(e2.assoc{k})
    x = e2.assoc{k};
    for q = 1:numel(x.id)
        local = x.meas_index(q);
        if local < 1 || local > numel(residual_map{k}), continue; end
        mi = residual_map{k}(local);
        if q <= numel(x.type) && startsWith(x.type{q}, 'companion_passive_')
            from_2d = strcmp(x.type{q}, 'companion_passive_2d');
            companion = companion_for_logical(companions, x.id(q), from_2d);
            id = logical_id_for_external(NaN, companion, offset);
            if isempty(companion)
                branch_3d_id = NaN;
                branch_2d_id = NaN;
            else
                branch_3d_id = field_or(companion, 'branch_3d_id', NaN);
                branch_2d_id = field_or(companion, 'branch_2d_id', NaN);
            end
        else
            id = x.id(q) + offset;
            branch_3d_id = NaN;
            branch_2d_id = x.id(q);
        end
        assoc_type = 'passive';
        if q <= numel(x.type) && ischar(x.type{q}) && ...
                strcmp(x.type{q}, 'passive_birth')
            assoc_type = 'passive_birth';
        end
        [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(x, q);
        filter_dim = indexed(x, 'filter_dim', q, 2);
        input_dim = indexed(x, 'input_dim', q, filter_dim);
        a = append_assoc(a, id, assoc_type, mi, events(k).passive.ids(mi), ...
            events(k).passive.ang(:, mi), nan(3, 1), ...
            indexed(x, 'cost', q, NaN), filter_dim, innovation, nis, ...
            group_size, innovation_kind, input_dim, branch_3d_id, branch_2d_id);
    end
end
end

function ids = committed_logical_ids(external_ids, companions, offset)
ids = reshape(external_ids, 1, []);
if isempty(ids) || isempty(companions), return; end
companion_index = lookup_companion_indices(ids, companions, 'external_3d_id');
for q = find(companion_index > 0)
    c = companions(companion_index(q));
    if c.from_2d && isfinite(c.local_2d_id)
        ids(q) = c.local_2d_id + offset;
    elseif isfinite(c.logical_id) && c.logical_id > 0
        ids(q) = c.logical_id;
    end
end
end

function branch_id = paired_2d_branch_for_external(external_id, companions)
branch_id = NaN;
if isempty(companions) || ~isfinite(external_id), return; end
index = lookup_companion_indices(external_id, companions, 'external_3d_id');
if isempty(index) || index(1) <= 0
    index = lookup_companion_indices( ...
        external_id, companions, 'pending_external_3d_id');
end
if ~isempty(index) && index(1) > 0
    companion = companions(index(1));
    branch_id = field_or(companion, 'branch_2d_id', NaN);
    if ~isfinite(branch_id)
        branch_id = field_or(companion, 'local_2d_id', NaN);
    end
end
end

function companions = companions_at(est2, k)
companions = repmat(companion_template(), 0, 1);
if isfield(est2, 'companions') && k <= numel(est2.companions) && ...
        ~isempty(est2.companions{k})
    companions = est2.companions{k};
end
end

function companions = decorate_companion_transactions(companions, offset)
for i = 1:numel(companions)
    q = companions(i);
    companions(i).branch_3d_id = q.external_3d_id;
    companions(i).branch_2d_id = q.local_2d_id;
    if q.from_2d && isfinite(q.local_2d_id)
        companions(i).logical_id = q.local_2d_id + offset;
    elseif isfinite(q.external_3d_id)
        companions(i).logical_id = q.external_3d_id;
    end
    if ~isfield(q, 'active_output_dim') || q.active_output_dim == 0
        if startsWith(q.mode, '3d')
            companions(i).active_output_dim = 3;
        elseif startsWith(q.mode, '2d')
            companions(i).active_output_dim = 2;
        end
    end
    if ~isfield(q, 'switch_state') || isempty(q.switch_state)
        if isfinite(q.pending_external_3d_id)
            companions(i).switch_state = 'pending_2d_to_3d';
        elseif companions(i).active_output_dim == 3
            companions(i).switch_state = 'stable_3d';
        elseif companions(i).active_output_dim == 2
            companions(i).switch_state = 'stable_2d';
        else
            companions(i).switch_state = 'hold_recovery';
        end
    end
end
end

function validate_transaction_event(out, companions, event_index)
ids = reshape([out.id], 1, []);
if numel(unique(ids)) ~= numel(ids)
    error('run_filter_joint_legacy_backbone:DuplicateFormalOutput', ...
        'Event %d contains duplicate formal output for one logical ID.', ...
        event_index);
end
for i = 1:numel(companions)
    q = companions(i);
    if startsWith(q.switch_state, 'pending_') && ...
            ~ismember(q.active_output_dim, [2, 3])
        error('run_filter_joint_legacy_backbone:InvalidPendingOwner', ...
            'Event %d pending transaction %g has no active formal owner.', ...
            event_index, q.logical_id);
    end
    if q.output_dim > 0 && q.output_dim ~= q.active_output_dim
        error('run_filter_joint_legacy_backbone:NonAtomicSwitch', ...
            ['Event %d logical ID %g exposes dimension %d while committed ' ...
             'active_output_dim is %d.'], event_index, q.logical_id, ...
            q.output_dim, q.active_output_dim);
    end
end
end

function stats = summarize_switch_transactions(records)
stats = struct('pending_3d_to_2d_event_records', 0, ...
    'pending_2d_to_3d_event_records', 0, ...
    'stable_3d_event_records', 0, 'stable_2d_event_records', 0, ...
    'hold_recovery_event_records', 0);
for k = 1:numel(records)
    q = records{k};
    if isempty(q), continue; end
    states = {q.switch_state};
    stats.pending_3d_to_2d_event_records = ...
        stats.pending_3d_to_2d_event_records + ...
        nnz(strcmp(states, 'pending_3d_to_2d'));
    stats.pending_2d_to_3d_event_records = ...
        stats.pending_2d_to_3d_event_records + ...
        nnz(strcmp(states, 'pending_2d_to_3d'));
    stats.stable_3d_event_records = stats.stable_3d_event_records + ...
        nnz(strcmp(states, 'stable_3d'));
    stats.stable_2d_event_records = stats.stable_2d_event_records + ...
        nnz(strcmp(states, 'stable_2d'));
    stats.hold_recovery_event_records = ...
        stats.hold_recovery_event_records + ...
        nnz(strcmp(states, 'hold_recovery'));
end
end

function log = map_manager_transition_log(raw, records, times, offset)
template = struct('id', 0, 't_sec', NaN, 'from', '', 'to', '', 'reason', '');
log = repmat(template, 0, 1);
if isempty(raw) || ~isstruct(raw), return; end
for q = 1:numel(raw)
    item = template;
    item.id = field_or(raw(q), 'id', 0) + offset;
    item.t_sec = field_or(raw(q), 't_sec', NaN);
    item.from = field_or(raw(q), 'from', '');
    item.to = field_or(raw(q), 'to', '');
    item.reason = field_or(raw(q), 'reason', '');
    if isfinite(item.t_sec) && ~isempty(times)
        [~, k] = min(abs(times - item.t_sec));
        candidates = records{k};
        if ~isempty(candidates)
            i = find([candidates.local_2d_id] == field_or(raw(q), 'id', 0), 1);
            if ~isempty(i), item.id = candidates(i).logical_id; end
        end
    end
    log(end + 1, 1) = item; %#ok<AGROW>
end
end

function q = companion_for_logical(companions, logical_id, from_2d)
q = repmat(companion_template(), 0, 1);
if isempty(companions) || ~isfinite(logical_id), return; end
i = find([companions.local_2d_id] == logical_id & ...
    [companions.from_2d] == logical(from_2d), 1);
if ~isempty(i), q = companions(i); end
end

function id = logical_id_for_external(external_id, companion, offset)
id = external_id;
if ~isempty(companion) && isfinite(companion.logical_id) && companion.logical_id > 0
    if companion.from_2d && isfinite(companion.local_2d_id)
        id = companion.local_2d_id + offset;
    else
        id = companion.logical_id;
    end
end
end

function birth_event = logical_birth_event(external_birth_event, companion)
birth_event = external_birth_event;
if ~isempty(companion) && ...
        isfinite(companion.birth_event) && companion.birth_event > 0
    birth_event = companion.birth_event;
end
end

function [out, n_suppressed] = arbitrate_logical_outputs(out)
n_suppressed = 0;
if numel(out) < 2, return; end
% Public identity is scoped by (dimension, branch), not by the temporary
% logical owner.  During a committed owner hand-off, the old and new owners
% can legitimately coexist in the manager snapshot while referring to the
% same physical 2-D/3-D branch.  Collapse that alias before public IDs are
% assigned, otherwise both rows receive the same branch-scoped public ID.
keep = true(1, numel(out));
visited = false(1, numel(out));
for q = 1:numel(out)
    if visited(q), continue; end
    dim = out(q).output_dim;
    if dim == 3
        branch_id = out(q).branch_3d_id;
        branches = [out.branch_3d_id];
    elseif dim == 2
        branch_id = out(q).branch_2d_id;
        branches = [out.branch_2d_id];
    else
        branch_id = NaN;
        branches = nan(1, numel(out));
    end
    if isfinite(branch_id) && branch_id > 0
        idx = find([out.output_dim] == dim & branches == branch_id);
    else
        idx = find([out.output_dim] == dim & [out.id] == out(q).id);
    end
    visited(idx) = true;
    if numel(idx) < 2, continue; end
    freshness = [out(idx).last_update_t];
    freshness(~isfinite(freshness)) = -inf;
    score = freshness + 1e-9 * [out(idx).output_dim];
    [~, best] = max(score);
    idx(best) = [];
    keep(idx) = false;
    n_suppressed = n_suppressed + numel(idx);
end
out = out(keep);
end

function [x, P, birth_event, found] = external_track_state(e3, k, external_id)
x = nan(9, 1); P = nan(9); birth_event = 0; found = false;
if k > numel(e3.tracks) || isempty(e3.tracks{k}), return; end
s = e3.tracks{k};
if ~isfield(s, 'L') || isempty(s.L), return; end
i = find(s.L(2, :) == external_id, 1);
if isempty(i) || size(s.m, 2) < i || size(s.P, 3) < i, return; end
x = s.m(:, i); P = s.P(:, :, i); birth_event = s.L(1, i);
found = all(isfinite(x)) && all(isfinite(P(:)));
end

function ids = legacy_assoc_ids(est, k)
ids = zeros(1, 0);
if k <= numel(est.assoc) && ~isempty(est.assoc{k}), ids = unique(est.assoc{k}.id); end
end

function ids = passive_update_ids(est, k)
ids = zeros(1, 0);
if k <= numel(est.passive_assoc) && ~isempty(est.passive_assoc{k})
    ids = unique(est.passive_assoc{k}.updated_track_ids);
end
end

function ids = output_ids(est)
nonempty = ~cellfun('isempty', est.output);
if ~any(nonempty)
    ids = zeros(1, 0);
    return;
end
chunks = cellfun(@(out) reshape([out.id], 1, []), ...
    est.output(nonempty), 'UniformOutput', false);
ids = unique([chunks{:}]);
end

function registry = init_dimension_output_registry(offset)
record = struct('internal_logical_id', NaN, ...
    'internal_logical_ids', zeros(1, 0), 'branch_id', NaN, ...
    'output_dim', 0, 'output_id', NaN, 'first_output_event', NaN, ...
    'last_output_event', NaN, 'first_output_t', NaN, ...
    'last_output_t', NaN, 'n_output_points', 0);
registry = struct('records', repmat(record, 0, 1), ...
    'slot_3d', zeros(1, 0), 'slot_2d', zeros(1, 0), ...
    'next_3d', 1, 'next_2d', offset + 1, 'offset', offset);
end

function [out, registry] = assign_dimension_output_ids(out, registry, event_index, event_t)
% Assign public IDs at the first formal output. The order is identical to
% the former whole-history pre-scan, but the half-million-point output
% history no longer needs a second rewrite after assembly.
for q = 1:numel(out)
    dim = out(q).output_dim;
    internal_id = out(q).logical_id;
    if ~isfinite(internal_id) || internal_id <= 0, internal_id = out(q).id; end
    internal_birth = out(q).birth_event;
    if dim == 3, branch_id = out(q).branch_3d_id;
    elseif dim == 2, branch_id = out(q).branch_2d_id;
    else, branch_id = NaN;
    end
    if ~ismember(dim, [2, 3]) || ~isfinite(branch_id) || branch_id <= 0 || ...
            branch_id ~= round(branch_id)
        error('run_filter_joint_legacy_backbone:MissingDimensionBranchId', ...
            '事件%d的正式%dD输出缺少有效正整数维度分支ID。', event_index, dim);
    end
    branch_index = round(branch_id);
    if dim == 3
        if branch_index > numel(registry.slot_3d), registry.slot_3d(branch_index) = 0; end
        slot = registry.slot_3d(branch_index);
    else
        if branch_index > numel(registry.slot_2d), registry.slot_2d(branch_index) = 0; end
        slot = registry.slot_2d(branch_index);
    end
    output_t = out(q).t_sec;
    if ~isfinite(output_t), output_t = event_t; end
    if slot == 0
        item = struct('internal_logical_id', internal_id, ...
            'internal_logical_ids', internal_id, 'branch_id', branch_id, ...
            'output_dim', dim, 'output_id', NaN, ...
            'first_output_event', event_index, 'last_output_event', event_index, ...
            'first_output_t', output_t, 'last_output_t', output_t, ...
            'n_output_points', 1);
        if dim == 3
            if registry.next_3d > registry.offset
                error('run_filter_joint_legacy_backbone:ThreeDimensionalIdOverflow', ...
                    ['三维正式输出ID数量超过命名空间上界%d；请增大' ...
                     'cfg.joint_2d_id_offset。'], registry.offset);
            end
            item.output_id = registry.next_3d;
            registry.next_3d = registry.next_3d + 1;
        else
            item.output_id = registry.next_2d;
            registry.next_2d = registry.next_2d + 1;
        end
        registry.records(end + 1, 1) = item; %#ok<AGROW>
        slot = numel(registry.records);
        if dim == 3, registry.slot_3d(branch_index) = slot;
        else, registry.slot_2d(branch_index) = slot;
        end
    else
        aliases = registry.records(slot).internal_logical_ids;
        if internal_id ~= registry.records(slot).internal_logical_id && ...
                ~any(aliases == internal_id)
            registry.records(slot).internal_logical_ids(end + 1) = internal_id;
        end
        registry.records(slot).last_output_event = event_index;
        registry.records(slot).last_output_t = output_t;
        registry.records(slot).n_output_points = ...
            registry.records(slot).n_output_points + 1;
    end
    out(q).internal_logical_id = internal_id;
    out(q).internal_birth_event = internal_birth;
    out(q).id = registry.records(slot).output_id;
    out(q).birth_event = registry.records(slot).first_output_event;
end
end

function out = compact_runtime_outputs(out)
% Keep the formal output/identity contract used by evaluation and review,
% while dropping matrices already retained inside the mature filter loop.
remove = {'angle_state', 'angle_cov', 'state3d', 'cov3d', ...
    'acceleration_enu', 'quality', 'shadow_for_logical_id'};
remove = intersect(remove, fieldnames(out), 'stable');
if ~isempty(remove), out = rmfield(out, remove); end
end

function est = remap_joint_association_ids(est, records, offset)
% Associations before confirmation are back-filled after all formal branch
% IDs are known. This preserves the established evaluation semantics.
slot_3d = zeros(1, 0); slot_2d = zeros(1, 0);
for q = 1:numel(records)
    branch = round(records(q).branch_id);
    if records(q).output_dim == 3
        if branch > numel(slot_3d), slot_3d(branch) = 0; end
        slot_3d(branch) = q;
    elseif records(q).output_dim == 2
        if branch > numel(slot_2d), slot_2d(branch) = 0; end
        slot_2d(branch) = q;
    end
end
for k = 1:numel(est.assoc)
    if isempty(est.assoc{k}), continue; end
    a = est.assoc{k};
    internal_ids = reshape(a.id, 1, []);
    a.internal_logical_id = internal_ids;
    a.public_branch_id = nan(size(internal_ids));
    a.public_3d_track_id = nan(size(internal_ids));
    a.public_2d_track_id = nan(size(internal_ids));
    for q = 1:numel(internal_ids)
        dim = indexed(a, 'filter_dim', q, 0);
        branch_3d_id = indexed(a, 'branch_3d_id', q, NaN);
        branch_2d_id = indexed(a, 'branch_2d_id', q, NaN);
        a.public_3d_track_id(q) = public_id_for_branch( ...
            branch_3d_id, 3, slot_3d, slot_2d, records, offset);
        a.public_2d_track_id(q) = public_id_for_branch( ...
            branch_2d_id, 2, slot_3d, slot_2d, records, offset);
        branch_id = association_branch_id(a, q, dim);
        public_id = NaN;
        if isfinite(branch_id) && branch_id > 0 && branch_id == round(branch_id)
            branch = round(branch_id); slot = 0;
            if dim == 3 && branch <= numel(slot_3d), slot = slot_3d(branch);
            elseif dim == 2 && branch <= numel(slot_2d), slot = slot_2d(branch);
            end
            if slot > 0, public_id = records(slot).output_id; end
        else
            [public_id, branch_id] = output_map_lookup_internal( ...
                records, internal_ids(q), dim);
        end
        a.public_branch_id(q) = branch_id;
        if isfinite(public_id) && public_id > 0
            a.id(q) = public_id;
        elseif isfinite(internal_ids(q)) && ismember(dim, [2, 3])
            a.id(q) = nonformal_output_id(branch_id, internal_ids(q), dim, offset);
        else
            a.id(q) = NaN;
        end
    end
    est.assoc{k} = a;
    if isfield(est, 'measurement_disposition') && ...
            k <= numel(est.measurement_disposition) && ...
            ~isempty(est.measurement_disposition{k})
        est.measurement_disposition{k} = remap_measurement_disposition_public( ...
            est.measurement_disposition{k}, a);
    end
end
end

function public_id = public_id_for_branch( ...
        branch_id, dim, slot_3d, slot_2d, records, offset)
public_id = NaN;
if ~isfinite(branch_id) || branch_id <= 0 || branch_id ~= round(branch_id)
    return;
end
branch = round(branch_id); slot = 0;
if dim == 3 && branch <= numel(slot_3d)
    slot = slot_3d(branch);
elseif dim == 2 && branch <= numel(slot_2d)
    slot = slot_2d(branch);
end
if slot > 0
    public_id = records(slot).output_id;
else
    public_id = nonformal_output_id(branch_id, branch_id, dim, offset);
end
end

function disposition = remap_measurement_disposition_public(disposition, a)
for name = {'active', 'passive'}
    type = name{1};
    if ~isfield(disposition, type) || ...
            ~isfield(disposition.(type), 'track_id')
        continue;
    end
    row = disposition.(type);
    row.internal_logical_id = row.track_id;
    row.public_branch_id = nan(size(row.track_id));
    q = find(strncmp(a.type, type, numel(type)));
    for j = reshape(q, 1, [])
        mi = a.meas_index(j);
        if ~isfinite(mi) || mi < 1 || mi > numel(row.track_id) || ...
                mi ~= round(mi)
            continue;
        end
        row.track_id(mi) = a.id(j);
        row.public_branch_id(mi) = a.public_branch_id(j);
    end
    disposition.(type) = row;
end
end

function branch_id = association_branch_id(a, q, dim)
branch_id = NaN;
if dim == 3
    branch_id = indexed(a, 'branch_3d_id', q, NaN);
elseif dim == 2
    branch_id = indexed(a, 'branch_2d_id', q, NaN);
end
end

function [id, branch_id] = output_map_lookup_internal(records, internal_id, dim)
id = NaN; branch_id = NaN;
if isempty(records) || ~isfinite(internal_id) || ~ismember(dim, [2, 3]), return; end
matches = zeros(1, 0);
for q = 1:numel(records)
    if records(q).output_dim == dim && ...
            ismember(internal_id, records(q).internal_logical_ids)
        matches(end + 1) = q; %#ok<AGROW>
    end
end
matches = unique(matches);
if numel(matches) == 1
    id = records(matches).output_id;
    branch_id = records(matches).branch_id;
end
end

function id = nonformal_output_id(branch_id, internal_id, dim, offset)
if isfinite(branch_id) && branch_id > 0
    base = max(1, round(abs(branch_id)));
else
    base = max(1, round(abs(internal_id)));
end
if dim == 3
    id = -base;
else
    id = -(offset + base);
end
end

function validate_dimension_output_ids(est, offset)
seen3 = false(1, offset);
seen2 = false(1, 0);
for k = 1:numel(est.output)
    out = est.output{k};
    if ~isempty(out)
        event_ids = reshape([out.id], 1, []);
        dims = reshape([out.output_dim], 1, []);
        if numel(unique(event_ids)) ~= numel(event_ids)
            error('run_filter_joint_legacy_backbone:DuplicatePublicOutputId', ...
                '事件%d包含重复的正式公开输出ID。', k);
        end
        event3 = event_ids(dims == 3);
        event2 = event_ids(dims == 2);
        if any(~isfinite(event3) | event3 ~= round(event3) | ...
                event3 < 1 | event3 > offset) || ...
                any(~isfinite(event2) | event2 ~= round(event2) | ...
                event2 <= offset)
            error('run_filter_joint_legacy_backbone:OutputIdNamespaceViolation', ...
                '事件%d的正式输出ID未遵守二维/三维命名空间。', k);
        end
        if ~isempty(event3), seen3(event3) = true; end
        if ~isempty(event2)
            local2 = event2 - offset;
            if max(local2) > numel(seen2), seen2(max(local2)) = false; end
            seen2(local2) = true;
        end
    end
    if isfield(est, 'assoc') && k <= numel(est.assoc) && ~isempty(est.assoc{k})
        a = est.assoc{k};
        positive = isfinite(a.id) & a.id > 0;
        invalid3 = positive & a.filter_dim == 3 & a.id > offset;
        invalid2 = positive & a.filter_dim == 2 & a.id <= offset;
        if any(invalid3 | invalid2)
            error('run_filter_joint_legacy_backbone:AssociationIdNamespaceViolation', ...
                '事件%d的关联记录未遵守二维/三维公开ID分域。', k);
        end
    end
end
ids3 = find(seen3);
ids2 = offset + find(seen2);
if ~isequal(ids3, 1:numel(ids3)) || ...
        ~isequal(ids2, offset + (1:numel(ids2)))
    error('run_filter_joint_legacy_backbone:NonContiguousOutputIds', ...
        '二维或三维正式输出ID不是按首次正式输出连续递增。');
end
end

function audit = build_output_identity_audit(est, mature3d, records, cfg)
% Reconcile mature external IDs, dimension branches, public IDs and the
% plotting lifetime filter without treating these different sets as one
% quantity.
min_life = max(1, round(get_cfg(cfg, 'joint_plot_min_life', 3)));
mature_ids = legacy_output_ids(mature3d);
r3 = records([records.output_dim] == 3);
r2 = records([records.output_dim] == 2);
branch3 = reshape([r3.branch_id], 1, []);
public3 = reshape([r3.output_id], 1, []);
public2 = reshape([r2.output_id], 1, []);
count3 = count_valid_public_output_points(est, public3, 3);
count2 = count_valid_public_output_points(est, public2, 2);
manager_only = setdiff(branch3, mature_ids, 'stable');
mature_without_joint = setdiff(mature_ids, branch3, 'stable');
alias_count = zeros(1, numel(r3));
rows = repmat(struct('public_id', NaN, 'branch_3d_id', NaN, ...
    'internal_logical_ids', zeros(1, 0), 'n_internal_logical_ids', 0, ...
    'n_3d_output_points', 0, 'first_3d_event', NaN, 'last_3d_event', NaN, ...
    'first_3d_t', NaN, 'last_3d_t', NaN, 'is_mature_external', false, ...
    'is_plot_visible', false, 'source', ''), numel(r3), 1);
for q = 1:numel(r3)
    aliases = unique(r3(q).internal_logical_ids, 'stable');
    alias_count(q) = numel(aliases);
    rows(q).public_id = r3(q).output_id;
    rows(q).branch_3d_id = r3(q).branch_id;
    rows(q).internal_logical_ids = aliases;
    rows(q).n_internal_logical_ids = numel(aliases);
    rows(q).n_3d_output_points = count3(q);
    rows(q).first_3d_event = r3(q).first_output_event;
    rows(q).last_3d_event = r3(q).last_output_event;
    rows(q).first_3d_t = r3(q).first_output_t;
    rows(q).last_3d_t = r3(q).last_output_t;
    rows(q).is_mature_external = ismember(r3(q).branch_id, mature_ids);
    rows(q).is_plot_visible = count3(q) >= min_life;
    if rows(q).is_mature_external
        rows(q).source = 'mature_3d_branch';
    else
        rows(q).source = 'manager_early_3d_branch';
    end
end
audit = struct('basis', 'dimension_branch_identity_v1', ...
    'plot_min_life', min_life, ...
    'n_mature_3d_external_ids', numel(mature_ids), ...
    'mature_3d_external_ids', mature_ids, ...
    'n_joint_3d_branch_ids', numel(branch3), ...
    'joint_3d_branch_ids', branch3, ...
    'n_public_3d_ids', numel(public3), 'public_3d_ids', public3, ...
    'n_plot_visible_3d_ids', nnz(count3 >= min_life), ...
    'plot_visible_3d_ids', public3(count3 >= min_life), ...
    'short_3d_public_ids', public3(count3 < min_life), ...
    'n_manager_only_3d_branches', numel(manager_only), ...
    'manager_only_3d_branch_ids', manager_only, ...
    'n_mature_without_joint_3d_output', numel(mature_without_joint), ...
    'mature_without_joint_3d_ids', mature_without_joint, ...
    'n_internal_owner_aliases_collapsed', sum(max(alias_count - 1, 0)), ...
    'n_public_2d_ids', numel(public2), 'public_2d_ids', public2, ...
    'n_plot_visible_2d_ids', nnz(count2 >= min_life), ...
    'plot_visible_2d_ids', public2(count2 >= min_life), ...
    'short_2d_public_ids', public2(count2 < min_life), ...
    'three_d_rows', rows);
end

function counts = count_valid_public_output_points(est, ids, dim)
counts = zeros(size(ids));
if isempty(ids), return; end
slot_by_id = zeros(1, max(ids));
slot_by_id(ids) = 1:numel(ids);
for k = 1:numel(est.output)
    out = est.output{k};
    if isempty(out), continue; end
    keep = [out.output_dim] == dim;
    candidates = out(keep);
    if isempty(candidates), continue; end
    candidate_ids = [candidates.id];
    valid_id = isfinite(candidate_ids) & candidate_ids == round(candidate_ids) & ...
        candidate_ids >= 1 & candidate_ids <= numel(slot_by_id);
    location = zeros(size(candidate_ids));
    location(valid_id) = slot_by_id(candidate_ids(valid_id));
    found = location > 0;
    if dim == 3
        valid = all(isfinite([candidates.position_enu]), 1);
    else
        valid = isfinite([candidates.az_deg]) & isfinite([candidates.el_deg]);
    end
    accepted = found & valid;
    if any(accepted)
        counts = counts + accumarray(location(accepted).', 1, ...
            [numel(ids), 1]).';
    end
end
end

function log = map_output_transition_log(raw, records)
template = struct('id', 0, 't_sec', NaN, 'from', '', 'to', '', 'reason', '');
log = repmat(template, 0, 1);
for q = 1:numel(raw)
    dim = mode_output_dimension(field_or(raw(q), 'to', ''));
    if dim == 0, dim = mode_output_dimension(field_or(raw(q), 'from', '')); end
    id = output_map_lookup_internal(records, field_or(raw(q), 'id', NaN), dim);
    if ~isfinite(id) || id <= 0, continue; end
    item = template;
    item.id = id;
    item.t_sec = field_or(raw(q), 't_sec', NaN);
    item.from = field_or(raw(q), 'from', '');
    item.to = field_or(raw(q), 'to', '');
    item.reason = field_or(raw(q), 'reason', '');
    log(end + 1, 1) = item; %#ok<AGROW>
end
end

function dim = mode_output_dimension(mode)
dim = 0;
if ~ischar(mode), return; end
if contains(lower(mode), '3d')
    dim = 3;
elseif contains(lower(mode), '2d')
    dim = 2;
end
end

function ids = legacy_output_ids(est)
nonempty = ~cellfun('isempty', est.L);
if ~any(nonempty)
    ids = zeros(1, 0);
    return;
end
chunks = cellfun(@(L) reshape(L(:, 2), 1, []), ...
    est.L(nonempty), 'UniformOutput', false);
ids = unique([chunks{:}]);
end

function m = maximum_legacy_id(est)
ids = legacy_output_ids(est);
m = 0;
if ~isempty(ids), m = max(ids); end
for k = 1:numel(est.tracks)
    if ~isempty(est.tracks{k}) && isfield(est.tracks{k}, 'L')
        values = est.tracks{k}.L(2, :);
        if ~isempty(values), m = max(m, max(values)); end
    end
end
end

function s = outputs_to_snapshots(out)
s = repmat(snapshot_template(), numel(out), 1);
for i = 1:numel(out)
    s(i).id = out(i).id; s(i).birth_event = out(i).birth_event;
    s(i).internal_birth_event = field_or(out(i), 'internal_birth_event', ...
        out(i).birth_event);
    s(i).internal_logical_id = field_or(out(i), 'internal_logical_id', ...
        field_or(out(i), 'logical_id', NaN));
    s(i).confirmed = out(i).confirmed; s(i).mode = out(i).mode;
    if isfield(out(i), 'last_update_t')
        s(i).last_update_t = out(i).last_update_t;
    end
    s(i).truth_id = out(i).truth_id;
    s(i).angle_state = out(i).angle_state; s(i).angle_cov = out(i).angle_cov;
    s(i).state3d = out(i).state3d; s(i).cov3d = out(i).cov3d;
    s(i).status_code = out(i).status_code; s(i).quality = out(i).quality;
    s(i).active_output_dim = out(i).active_output_dim;
    s(i).switch_state = out(i).switch_state;
    s(i).branch_3d_id = out(i).branch_3d_id;
    s(i).branch_2d_id = out(i).branch_2d_id;
end
end

function log = build_transition_log(est)
log = repmat(struct('id', 0, 't_sec', NaN, 'from', '', 'to', '', 'reason', ''), 0, 1);
last_mode_by_id = cell(1, 0);
for k = 1:numel(est.output)
    for q = 1:numel(est.output{k})
        o = est.output{k}(q);
        id = round(o.id);
        if id > numel(last_mode_by_id), last_mode_by_id{id} = []; end
        if isempty(last_mode_by_id{id})
            last_mode_by_id{id} = o.mode;
        elseif ~strcmp(last_mode_by_id{id}, o.mode)
            log(end + 1) = struct('id', o.id, 't_sec', o.t_sec, ...
                'from', last_mode_by_id{id}, 'to', o.mode, ...
                'reason', 'status_change'); %#ok<AGROW>
            last_mode_by_id{id} = o.mode;
        end
    end
end
end

function est = init_joint_estimate(K, events)
est = struct('X', {cell(K, 1)}, 'P', {cell(K, 1)}, 'L', {cell(K, 1)}, ...
    'N', zeros(K, 1), 'X2', {cell(K, 1)}, 'P2', {cell(K, 1)}, ...
    'L2', {cell(K, 1)}, 'N2', zeros(K, 1), 'N_total', zeros(K, 1), ...
    'logical_tracks', {cell(K, 1)}, 'output', {cell(K, 1)}, ...
    'shadow_output', {cell(K, 1)}, 'switch_transactions', {cell(K, 1)}, ...
    'tracks', {cell(K, 1)}, 'companions', {cell(K, 1)}, ...
    'assoc', {cell(K, 1)}, 'measurement_disposition', {cell(K, 1)}, ...
    'filter_times', reshape([events.t_sec], [], 1), 'event_meta', events, ...
    'mode_counts', struct('n2d', zeros(K, 1), 'n3d', zeros(K, 1), ...
    'nhold', zeros(K, 1)), 'timing', struct('total', 0), ...
    'transition_log', []);
end

function o = output_template()
o = struct('id', 0, 'birth_event', 0, 'internal_birth_event', 0, ...
    't_sec', NaN, 'mode', '', ...
    'status_code', 0, 'output_dim', 0, 'confirmed', false, 'truth_id', NaN, ...
    'formal', true, 'logical_id', NaN, 'internal_logical_id', NaN, ...
    'branch_3d_id', NaN, ...
    'branch_2d_id', NaN, 'active_output_dim', 0, ...
    'switch_state', '', 'shadow_for_logical_id', NaN, ...
    'last_update_t', NaN, ...
    'az_deg', NaN, 'el_deg', NaN, 'range_m', NaN, ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9), ...
    'position_enu', nan(3, 1), 'velocity_enu', nan(3, 1), ...
    'acceleration_enu', nan(3, 1), 'quality', struct());
end

function s = snapshot_template()
s = struct('id', 0, 'birth_event', 0, 'internal_birth_event', 0, ...
    'confirmed', false, 'mode', '', ...
    'status_code', 0, 'last_update_t', NaN, 'truth_id', NaN, ...
    'internal_logical_id', NaN, ...
    'active_output_dim', 0, 'switch_state', '', ...
    'branch_3d_id', NaN, 'branch_2d_id', NaN, ...
    'quality', struct(), 'hit_history', zeros(1, 0), ...
    'angle_state', nan(6, 1), 'angle_cov', nan(6), ...
    'state3d', nan(9, 1), 'cov3d', nan(9));
end

function s = companion_template()
s = struct('external_3d_id', NaN, 'local_2d_id', NaN, ...
    'logical_id', NaN, 'previous_external_3d_id', NaN, ...
    'pending_external_3d_id', NaN, ...
    'branch_3d_id', NaN, 'branch_2d_id', NaN, ...
    'active_output_dim', 0, 'switch_state', 'hold_recovery', ...
    'pending_target_branch_id', NaN, 'switch_candidate_since', NaN, ...
    'switch_commit_t', NaN, ...
    'from_2d', false, 'confirmed', false, 'mode', '', 'output_dim', 0, ...
    'external_fresh', false, 'birth_event', 0, 'birth_t', NaN, ...
    'last_update_t', NaN, 'last_active_t', NaN, ...
    'last_valid_range_t', NaN, ...
    'active_evidence_count', 0, 'active_evidence_window', 0, ...
    'up_count', 0, 'down_count', 0, ...
    'angle_state', nan(6, 1), ...
    'angle_cov', nan(6), 'quality', struct());
end

function e = event_template()
e = struct('cycle_id', 0, 't_sec', NaN, 't_start', NaN, 't_end', NaN, ...
    'confirm_cycle', false, ...
    'has_active', false, 'has_passive', false, ...
    'active', active_template(), 'passive', passive_template());
end

function a = active_template()
a = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'R_xyz', zeros(3, 3, 0), 'R_ae', zeros(2, 2, 0), ...
    'has_range', false(1, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
end

function p = passive_template()
p = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), ...
    'R_ae', zeros(2, 2, 0), 'ids', zeros(1, 0), 'src', zeros(1, 0), ...
    'kind', zeros(1, 0), 'n_meas', 0);
end

function a = assoc_template()
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, ...
    'meas_index', zeros(1, 0), 'tid', zeros(1, 0), ...
    'ang', zeros(2, 0), 'xyz', zeros(3, 0), 'cost', zeros(1, 0), ...
    'measurement_dim', zeros(1, 0), 'filter_dim', zeros(1, 0), ...
    'input_dim', zeros(1, 0), 'range_updated', false(1, 0), ...
    'branch_3d_id', zeros(1, 0), 'branch_2d_id', zeros(1, 0), ...
    'public_branch_id', zeros(1, 0), ...
    'public_3d_track_id', zeros(1, 0), ...
    'public_2d_track_id', zeros(1, 0), ...
    'innovation', zeros(3, 0), ...
    'nis', zeros(1, 0), 'group_size', zeros(1, 0), ...
    'innovation_kind', zeros(1, 0));
end

function a = append_assoc(a, id, type, mi, tid, ang, xyz, cost, filter_dim, ...
        innovation, nis, group_size, innovation_kind, input_dim, ...
        branch_3d_id, branch_2d_id)
if nargin < 10 || isempty(innovation), innovation = nan(3, 1); end
if nargin < 11 || isempty(nis), nis = NaN; end
if nargin < 12 || isempty(group_size), group_size = 1; end
if nargin < 13 || isempty(innovation_kind), innovation_kind = 0; end
if nargin < 14 || isempty(input_dim), input_dim = filter_dim; end
if nargin < 15 || isempty(branch_3d_id), branch_3d_id = NaN; end
if nargin < 16 || isempty(branch_2d_id), branch_2d_id = NaN; end
a.id(end + 1) = id; a.type{end + 1} = type; a.meas_index(end + 1) = mi;
a.tid(end + 1) = tid; a.ang(:, end + 1) = ang; a.xyz(:, end + 1) = xyz;
a.cost(end + 1) = cost; a.filter_dim(end + 1) = filter_dim;
if strncmp(type, 'active', 6)
    a.measurement_dim(end + 1) = 3;
else
    a.measurement_dim(end + 1) = 2;
end
a.input_dim(end + 1) = input_dim;
a.range_updated(end + 1) = strncmp(type, 'active', 6) && filter_dim == 3;
a.branch_3d_id(end + 1) = branch_3d_id;
a.branch_2d_id(end + 1) = branch_2d_id;
a.public_branch_id(end + 1) = NaN;
a.public_3d_track_id(end + 1) = NaN;
a.public_2d_track_id(end + 1) = NaN;
a.innovation(:, end + 1) = sized_innovation(innovation);
a.nis(end + 1) = nis;
a.group_size(end + 1) = group_size;
a.innovation_kind(end + 1) = innovation_kind;
end

function disposition = build_measurement_disposition( ...
        a, event, event_index, active_reason, passive_reason)
active_suppressed = true(1, event.active.n_meas);
passive_suppressed = true(1, event.passive.n_meas);
for q = 1:numel(a.id)
    mi = a.meas_index(q);
    if ~isfinite(mi) || mi ~= round(mi), continue; end
    if strncmp(a.type{q}, 'active', 6) && mi >= 1 && mi <= numel(active_suppressed)
        active_suppressed(mi) = false;
    elseif strncmp(a.type{q}, 'passive', 7) && mi >= 1 && mi <= numel(passive_suppressed)
        passive_suppressed(mi) = false;
    end
end
disposition = struct( ...
    'active', joint_measurement_disposition(event.active.n_meas, a, ...
        'active', active_suppressed, event_index, active_reason), ...
    'passive', joint_measurement_disposition(event.passive.n_meas, a, ...
        'passive', passive_suppressed, event_index, passive_reason));
end

function stats = summarize_measurement_disposition(records)
stats = struct('active_measurements', 0, 'passive_measurements', 0, ...
    'active_assigned', 0, 'passive_assigned', 0, ...
    'active_births', 0, 'passive_births', 0, ...
    'active_birth_suppressed', 0, 'passive_birth_suppressed', 0, ...
    'active_unaccounted', 0, 'passive_unaccounted', 0, ...
    'active_range_measurements', 0, 'active_range_updates', 0, ...
    'active_range_births', 0, 'active_range_birth_suppressed', 0, ...
    'active_range_unaccounted', 0, ...
    'passive_to_2d', 0, 'passive_to_3d', 0, 'passive_not_routed', 0);
for k = 1:numel(records)
    if isempty(records{k}), continue; end
    active = records{k}.active;
    passive = records{k}.passive;
    stats.active_measurements = stats.active_measurements + numel(active.action);
    stats.passive_measurements = stats.passive_measurements + numel(passive.action);
    stats.active_assigned = stats.active_assigned + nnz(active.action == 1);
    stats.active_births = stats.active_births + nnz(active.action == 2);
    stats.active_birth_suppressed = stats.active_birth_suppressed + nnz(active.action == 3);
    stats.passive_assigned = stats.passive_assigned + nnz(passive.action == 1);
    stats.passive_births = stats.passive_births + nnz(passive.action == 2);
    stats.passive_birth_suppressed = stats.passive_birth_suppressed + nnz(passive.action == 3);
    stats.active_unaccounted = stats.active_unaccounted + nnz(active.action == 0);
    stats.passive_unaccounted = stats.passive_unaccounted + nnz(passive.action == 0);
    stats.passive_to_2d = stats.passive_to_2d + nnz(passive.input_dim == 2);
    stats.passive_to_3d = stats.passive_to_3d + nnz(passive.input_dim == 3);
    stats.passive_not_routed = stats.passive_not_routed + nnz(passive.input_dim == 0);
end
stats.active_range_measurements = stats.active_measurements;
stats.active_range_updates = stats.active_assigned + stats.active_births;
stats.active_range_births = stats.active_births;
stats.active_range_birth_suppressed = stats.active_birth_suppressed;
stats.active_range_unaccounted = stats.active_unaccounted;
end

function [innovation, nis, group_size, innovation_kind] = assoc_diagnostic(s, q)
innovation = nan(3, 1); nis = NaN; group_size = 1; innovation_kind = 0;
if isstruct(s) && isfield(s, 'innovation') && size(s.innovation, 2) >= q
    innovation = sized_innovation(s.innovation(:, q));
end
nis = indexed(s, 'nis', q, nis);
group_size = indexed(s, 'group_size', q, group_size);
innovation_kind = indexed(s, 'innovation_kind', q, innovation_kind);
end

function v = sized_innovation(x)
v = nan(3, 1);
if isempty(x), return; end
n = min(3, numel(x)); v(1:n) = x(1:n);
end

function C = normalize_covariance(C, dim, n, fallback)
if isempty(C), C = repmat(fallback, 1, 1, n); return; end
if ismatrix(C), C = repmat(C, 1, 1, n); end
if size(C, 3) < n, C(:, :, end + 1:n) = repmat(fallback, 1, 1, n - size(C, 3)); end
C = C(1:dim, 1:dim, 1:n);
end

function v = sized_row(v, n, fallback)
if isempty(v), v = fallback * ones(1, n); else, v = v(:).'; end
if isscalar(v) && n > 1, v = repmat(v, 1, n); end
if numel(v) < n, v = [v, fallback * ones(1, n - numel(v))]; end
v = v(1:n);
end

function M = sized_matrix(M, rows, cols, fallback)
if isempty(M), M = fallback * ones(rows, cols); return; end
if size(M, 1) < rows, M(end + 1:rows, :) = fallback; end
if size(M, 2) < cols, M(:, end + 1:cols) = fallback; end
M = M(1:rows, 1:cols);
end

function v = cell_value(c, k, fallback)
if iscell(c) && k <= numel(c) && ~isempty(c{k}), v = c{k}; else, v = fallback; end
end

function v = indexed(s, name, q, fallback)
if isstruct(s) && isfield(s, name) && numel(s.(name)) >= q
    v = s.(name)(q);
else
    v = fallback;
end
end

function v = field_or(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)), v = s.(name); else, v = fallback; end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function sensor = platform_enu(t, platform, cfg)
lat = platform.interp_lat(t); lon = platform.interp_lon(t); alt = platform.interp_alt(t);
origin = field_or(cfg, 'local_origin', 'first_platform');
if isnumeric(origin) && numel(origin) >= 3 && all(isfinite(origin(1:3)))
    lat0 = origin(1); lon0 = origin(2); alt0 = origin(3);
else
    lat0 = platform.lat_deg(1); lon0 = platform.lon_deg(1); alt0 = platform.alt_m(1);
end
sensor = ecef_to_enu_rot(lat0, lon0) * ...
    (llh_to_ecef(lat, lon, alt) - llh_to_ecef(lat0, lon0, alt0));
end

function ecef = llh_to_ecef(lat_deg, lon_deg, alt_m)
a = 6378137; e2 = 6.69437999014e-3; lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
N = a / sqrt(1 - e2 * sin(lat)^2);
ecef = [(N + alt_m) * cos(lat) * cos(lon); ...
    (N + alt_m) * cos(lat) * sin(lon); ...
    (N * (1 - e2) + alt_m) * sin(lat)];
end

function R = ecef_to_enu_rot(lat_deg, lon_deg)
lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
R = [-sin(lon), cos(lon), 0; ...
    -sin(lat) * cos(lon), -sin(lat) * sin(lon), cos(lat); ...
    cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)];
end
