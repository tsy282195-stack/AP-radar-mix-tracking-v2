function metrics = evaluate_joint_tracking_metrics(est, events, cfg)
%EVALUATE_JOINT_TRACKING_METRICS 分别评价二维、三维及分域公开输出ID。
%
% 关联率按物理量测维度划分。三维身份评价只使用有效距离RAE；二维
% 身份评价统一使用物理被动AE和主动AE-only。主动AE-only保持二维
% 量测身份，不进入三维位置/距离评价，也不在升维后重复计入三维量测。
% 分维量测指标严格区分：关联利用率=已关联/输入，量测正确率=正确关联/输入，
% 已关联一致率=正确关联/已关联。
% 总体评价只汇总独立二维/三维结果，不执行跨维Track<->Truth重匹配，
% 也不混合角度与位置的物理单位。

if nargin < 1 || isempty(est), est = struct(); end
if nargin < 2 || isempty(events), events = repmat(empty_event(), 0, 1); end
if nargin < 3 || isempty(cfg), cfg = struct(); end

truth_labels = build_joint_truth_labels(events, cfg);
data = collect_metric_data(est, events, truth_labels);
validate_output_id_domains(data, cfg);
confirmed_ids = collect_confirmed_ids(est, data.output_track);

metrics = struct();
metrics.mode = 'joint_2d3d';
metrics.evaluation_version = 8;
metrics.status = 'ok';
metrics.truth_targets = truth_labels.summary;
metrics.id_split = metrics.truth_targets;
metrics.two_d = build_scope_metrics(data, 2, confirmed_ids, cfg);
metrics.three_d = build_scope_metrics(data, 3, confirmed_ids, cfg);
% Overall is a sum of the two independently matched physical scopes. It is
% deliberately not a fourth cross-dimensional Track<->Truth assignment.
metrics.overall = combine_dimension_scopes(metrics.two_d, metrics.three_d);
metrics.measurement_flow = build_measurement_flow(data);
metrics.measurement_accounting = build_measurement_accounting(est, data);
metrics.track_details = build_track_details(data, metrics);
metrics.output_identity_audit = attach_evaluation_identity_audit(est, metrics);
metrics.contract_checks = assert_dimension_evaluation_contract(metrics);
metrics.measurement_consistency = struct( ...
    'status', 'ok', ...
    'reference', 'measurement_labels_and_active_measurement_positions', ...
    'two_d', metrics.two_d, 'three_d', metrics.three_d, ...
    'overall', metrics.overall);
% 保留原联合评价器字段，避免已有脚本失效。
metrics.association = metrics.overall.association;
metrics.angle = metrics.overall.angle;
metrics.position = metrics.three_d.position;
metrics.accuracy = metrics.overall.accuracy;
metrics.measurement_accuracy = metrics.overall.measurement_accuracy;
metrics.association_consistency = metrics.overall.association_consistency;
metrics.track_accuracy = metrics.overall.track_accuracy;
metrics.start_time = metrics.overall.start_time;
metrics.output_tracks = struct( ...
    'n_unique', metrics.overall.output.n_unique_tracks, ...
    'n_transitions', get_transition_count(est), ...
    'n_2d_outputs', metrics.two_d.output.n_outputs, ...
    'n_3d_outputs', metrics.three_d.output.n_outputs, ...
    'id_contract', sprintf('3d:1..N;2d:%d..', ...
        round(get_cfg(cfg, 'joint_2d_id_offset', 1000)) + 1));
metrics.logical_tracks = metrics.output_tracks; % 兼容旧调用；现为正式输出ID统计

if get_cfg(cfg, 'metrics_max_print', 12) > 0
    print_joint_report(metrics);
end
end

function data = collect_metric_data(est, events, labels)
K = numel(events);
n2_event = zeros(K, 1); n3_event = zeros(K, 1);
assoc_event = zeros(K, 1); output_event = zeros(K, 1);
for k = 1:K
    e = events(k); na = e.active.n_meas; np = e.passive.n_meas;
    has_range = measurement_has_range(e.active, na);
    n3_event(k) = nnz(has_range);
    n2_event(k) = np + na - n3_event(k);
    a = event_assoc(est, k); out = event_output(est, k);
    assoc_event(k) = numel(a.id);
    output_event(k) = numel(out);
end
data = preallocate_metric_data(sum(n2_event), sum(n3_event), ...
    sum(assoc_event), sum(output_event), n2_event, n3_event);
data.n_meas_2d = sum(n2_event);
data.n_meas_3d = sum(n3_event);
data.n_active_measurements = 0;
data.n_passive_measurements = 0;
data.n_active_ae_only_measurements = 0;
data.n_active_range_measurements = data.n_meas_3d;
p2 = 0; p3 = 0; pa = 0; po = 0;
for k = 1:K
    e = events(k);
    na = e.active.n_meas;
    np = e.passive.n_meas;
    has_range = measurement_has_range(e.active, na);

    active_ids = sized_row(labels.active{k}, na, NaN);
    passive_ids = sized_row(labels.passive{k}, np, NaN);
    active_t = measurement_times(e.active.t_sec, na, e.t_sec);
    passive_t = measurement_times(e.passive.t_sec, np, e.t_sec);
    passive_source = passive_measurement_sources(e.passive, np);
    n2 = n2_event(k); i2 = p2 + (1:n2);
    data.truth_2d_id(i2) = [active_ids(~has_range), passive_ids];
    data.truth_2d_t(i2) = [active_t(~has_range), passive_t];
    data.truth_2d_event(i2) = k;
    data.truth_2d_az(i2) = [e.active.rae(2, ~has_range), e.passive.ang(1, :)];
    data.truth_2d_el(i2) = [e.active.rae(3, ~has_range), e.passive.ang(2, :)];
    data.truth_2d_source(i2) = [2 * ones(1, nnz(~has_range)), passive_source];
    data.n_active_measurements = data.n_active_measurements + na + ...
        nnz(passive_source == 2);
    data.n_active_ae_only_measurements = ...
        data.n_active_ae_only_measurements + nnz(~has_range) + ...
        nnz(passive_source == 2);
    data.n_passive_measurements = data.n_passive_measurements + ...
        nnz(passive_source == 3);
    p2 = p2 + n2;

    n3 = n3_event(k); i3 = p3 + (1:n3);
    data.truth_3d_id(i3) = active_ids(has_range);
    data.truth_3d_t(i3) = active_t(has_range);
    data.truth_3d_event(i3) = k;
    data.truth_3d_az(i3) = e.active.rae(2, has_range);
    data.truth_3d_el(i3) = e.active.rae(3, has_range);
    data.truth_3d_xyz(:, i3) = e.active.xyz(:, has_range);
    p3 = p3 + n3;

    a = event_assoc(est, k);
    na_assoc = assoc_event(k); ia = pa + (1:na_assoc);
    filter_dim = association_filter_dimensions(a, est, k);
    input_dim = association_input_dimensions(a, filter_dim);
    active_input = measurement_input_dimensions(est, a, input_dim, k, 'active', na);
    passive_input = measurement_input_dimensions(est, a, input_dim, k, 'passive', np);
    data.truth_2d_input_dim(i2) = [active_input(~has_range), passive_input];
    data.truth_2d_is_passive(i2) = data.truth_2d_source(i2) == 3;
    if na_assoc > 0
        data.assoc_track(ia) = reshape(a.id, 1, []);
        data.assoc_filter_dim(ia) = filter_dim;
        data.assoc_input_dim(ia) = input_dim;
        data.assoc_event(ia) = k;
    end
    for q = 1:na_assoc
        j = pa + q;
        data.assoc_truth(j) = association_truth_label(a, q, e, labels, k);
        meas_dim = association_measurement_dim(a, q, e);
        data.assoc_meas_dim(j) = meas_dim;
        type = indexed_text(a, 'type', q, '');
        data.assoc_source(j) = association_source_kind(a, q, e, meas_dim);
        data.assoc_is_active(j) = ismember(data.assoc_source(j), [1, 2]);
        data.assoc_is_passive(j) = data.assoc_source(j) == 3;
        mi = round(indexed_field_value(a, 'meas_index', q, 0));
        if association_type_matches(type, 'active') && ...
                mi >= 1 && mi <= numel(active_t)
            data.assoc_t(j) = active_t(mi);
            data.assoc_input_dim(j) = active_input(mi);
        elseif association_type_matches(type, 'passive') && ...
                mi >= 1 && mi <= numel(passive_t)
            data.assoc_t(j) = passive_t(mi);
            data.assoc_input_dim(j) = passive_input(mi);
        else
            data.assoc_t(j) = e.t_sec;
        end
        % Legacy association schemas only recorded accepted associations;
        % for them, a 3-D active record implies a completed range update.
        range_updated = meas_dim == 3;
        if isfield(a, 'range_updated')
            range_updated = logical(indexed_value(a.range_updated, q, false));
        end
        data.assoc_range_updated(j) = range_updated;
    end
    pa = pa + na_assoc;

    out = event_output(est, k);
    nout = output_event(k); io = po + (1:nout);
    if nout > 0
        data.output_track(io) = reshape([out.id], 1, []);
        data.output_truth(io) = reshape([out.truth_id], 1, []);
        data.output_dim(io) = reshape([out.output_dim], 1, []);
        data.output_t(io) = reshape([out.t_sec], 1, []);
        data.output_event(io) = k;
        data.output_az(io) = reshape([out.az_deg], 1, []);
        data.output_el(io) = reshape([out.el_deg], 1, []);
        data.output_pos(:, io) = cat(2, out.position_enu);
    end
    po = po + nout;
end
end

function scope = build_scope_metrics(data, dim, confirmed_ids, cfg)
if dim == 2
    meas_mask = data.assoc_meas_dim == 2;
    filter_mask = data.assoc_filter_dim == 2;
    output_mask = data.output_dim == 2;
    name = 'two_d';
    n_meas = data.n_meas_2d;
elseif dim == 3
    meas_mask = data.assoc_meas_dim == 3;
    filter_mask = data.assoc_filter_dim == 3;
    output_mask = data.output_dim == 3;
    name = 'three_d';
    n_meas = data.n_meas_3d;
else
    meas_mask = data.assoc_meas_dim > 0;
    filter_mask = true(size(data.assoc_track));
    output_mask = data.output_dim > 0;
    name = 'overall';
    n_meas = data.n_meas_2d + data.n_meas_3d;
end

output_ids = unique(data.output_track(output_mask & isfinite(data.output_track)));
if dim == 2
    scoped_assoc_mask = meas_mask;
elseif dim == 3
    scoped_assoc_mask = meas_mask & data.assoc_input_dim == 3;
else
    scoped_assoc_mask = filter_mask;
end
[truth_id, ~, truth_t, reference_basis] = evaluation_truth_scope(data, dim);
scope = struct();
scope.name = name;
scope.reference = struct('basis', reference_basis, ...
    'n_labeled_measurements', nnz(isfinite(truth_id)));
scope.association = association_metrics(n_meas, data.assoc_track(meas_mask), confirmed_ids);
scope.association.n_mode_assigned = nnz(scoped_assoc_mask);
scope.accuracy = id_accuracy_metrics(data.assoc_track(scoped_assoc_mask), ...
    data.assoc_truth(scoped_assoc_mask), output_ids, truth_id, cfg);
scope.measurement_accuracy = measurement_accuracy_metrics( ...
    n_meas, scope.accuracy.n_correct);
scope.association_consistency = association_consistency_metrics( ...
    scope.association.n_assigned, scope.accuracy.n_correct);
scope.track_accuracy = scope.accuracy.track_level;
scope.start_time = start_delay_metrics(scope.accuracy, truth_id, truth_t, ...
    data.output_track(output_mask), data.output_t(output_mask), ...
    data.assoc_track(scoped_assoc_mask), data.assoc_truth(scoped_assoc_mask), ...
    data.assoc_t(scoped_assoc_mask));
[scope.angle, scope.position] = output_error_metrics( ...
    data, dim, output_mask, scope.accuracy.output_pairs);
scope.output_coverage = formal_output_coverage( ...
    data, dim, output_mask, scope.accuracy.output_pairs, output_ids);
scope.output = struct('n_outputs', nnz(output_mask), ...
    'n_unique_tracks', numel(output_ids), 'track_ids', output_ids);
if dim == 2
    passive = data.truth_2d_is_passive;
    active_ae = data.truth_2d_source == 2;
    scope.reference.n_passive_input = nnz(passive);
    scope.reference.n_passive_to_2d = nnz(passive & data.truth_2d_input_dim == 2);
    scope.reference.n_passive_to_3d = nnz(passive & data.truth_2d_input_dim == 3);
    scope.reference.n_passive_not_routed = nnz(passive & data.truth_2d_input_dim == 0);
    scope.reference.n_active_ae_only_input = nnz(active_ae);
    scope.reference.n_active_ae_only_to_2d = ...
        nnz(active_ae & data.truth_2d_input_dim == 2);
    scope.reference.n_active_ae_only_to_3d = ...
        nnz(active_ae & data.truth_2d_input_dim == 3);
    scope.reference.n_active_ae_only_not_routed = ...
        nnz(active_ae & data.truth_2d_input_dim == 0);
end
end

function scope = combine_dimension_scopes(two_d, three_d)
scope = struct();
scope.name = 'overall_dimension_separated';
scope.reference = struct('basis', 'sum_of_separate_2d_and_3d_references', ...
    'n_labeled_measurements', two_d.reference.n_labeled_measurements + ...
    three_d.reference.n_labeled_measurements);

a2 = two_d.association; a3 = three_d.association;
n_measurements = a2.n_measurements + a3.n_measurements;
n_assigned = a2.n_assigned + a3.n_assigned;
n_confirmed = a2.n_assigned_confirmed + a3.n_assigned_confirmed;
scope.association = struct('basis', 'dimension_separated_measurement_sum', ...
    'n_measurements', n_measurements, 'n_assigned', n_assigned, ...
    'n_assigned_confirmed', n_confirmed, ...
    'rate_all_tracks', safe_ratio(n_assigned, n_measurements), ...
    'rate_confirmed_tracks', safe_ratio(n_confirmed, n_measurements), ...
    'n_mode_assigned', a2.n_mode_assigned + a3.n_mode_assigned);

scope.accuracy = combine_accuracy(two_d.accuracy, three_d.accuracy);
scope.measurement_accuracy = combine_measurement_accuracy( ...
    two_d.measurement_accuracy, three_d.measurement_accuracy);
scope.association_consistency = combine_association_consistency( ...
    two_d.association_consistency, three_d.association_consistency);
scope.track_accuracy = scope.accuracy.track_level;
scope.start_time = combine_start_time(two_d.start_time, three_d.start_time);
scope.angle = combine_angle_metrics(two_d.angle, three_d.angle);
scope.position = combine_position_metrics(two_d.position, three_d.position);
scope.output_coverage = combine_output_coverage( ...
    two_d.output_coverage, three_d.output_coverage);
scope.output = struct('n_outputs', two_d.output.n_outputs + three_d.output.n_outputs, ...
    'n_unique_tracks', two_d.output.n_unique_tracks + three_d.output.n_unique_tracks, ...
    'n_2d_tracks', two_d.output.n_unique_tracks, ...
    'n_3d_tracks', three_d.output.n_unique_tracks, ...
    'track_ids', [reshape(three_d.output.track_ids, 1, []), ...
                  reshape(two_d.output.track_ids, 1, [])]);
scope.aggregate_contract = 'no_cross_dimension_rematching';
end

function result = combine_measurement_accuracy(a2, a3)
n_measurements = a2.n_measurements + a3.n_measurements;
n_correct = a2.n_correct + a3.n_correct;
result = measurement_accuracy_metrics(n_measurements, n_correct);
result.basis = 'dimension_separated_correct_associations_over_input_measurements';
end

function result = combine_association_consistency(a2, a3)
n_assigned = a2.n_assigned + a3.n_assigned;
n_correct = a2.n_correct + a3.n_correct;
result = association_consistency_metrics(n_assigned, n_correct);
result.basis = 'dimension_separated_correct_associations_over_assigned_measurements';
end

function acc = combine_accuracy(a2, a3)
acc = struct();
acc.n_assoc_total = a2.n_assoc_total + a3.n_assoc_total;
acc.n_labeled_assoc = a2.n_labeled_assoc + a3.n_labeled_assoc;
acc.n_correct = a2.n_correct + a3.n_correct;
acc.n_error = a2.n_error + a3.n_error;
acc.accuracy = safe_ratio(acc.n_correct, acc.n_labeled_assoc);
acc.n_truth = a2.n_truth + a3.n_truth;
acc.n_tracks = a2.n_tracks + a3.n_tracks;
acc.truth_ids = [reshape(a3.truth_ids, 1, []), reshape(a2.truth_ids, 1, [])];
acc.track_ids = [reshape(a3.track_ids, 1, []), reshape(a2.track_ids, 1, [])];
acc.confusion = blkdiag(a3.confusion, a2.confusion);
acc.pairs = [reshape(a3.pairs, [], 1); reshape(a2.pairs, [], 1)];
acc.fragmented_truth_count = a2.fragmented_truth_count + a3.fragmented_truth_count;
acc.mixed_track_count = a2.mixed_track_count + a3.mixed_track_count;
acc.output_pairs = [reshape(a3.output_pairs, [], 1); reshape(a2.output_pairs, [], 1)];
acc.output_labeled_counts = [reshape(a3.output_labeled_counts, 1, []), ...
    reshape(a2.output_labeled_counts, 1, [])];
acc.track_level = combine_track_accuracy(a2.track_level, a3.track_level);
end

function result = combine_track_accuracy(a2, a3)
ids = [reshape(a3.track_ids, 1, []), reshape(a2.track_ids, 1, [])];
coverage = [reshape(a3.coverage, 1, []), reshape(a2.coverage, 1, [])];
consistency = [reshape(a3.association_consistency, 1, []), ...
    reshape(a2.association_consistency, 1, [])];
correct = [reshape(a3.is_correct, 1, []), reshape(a2.is_correct, 1, [])];
result = struct('purity_threshold', a2.purity_threshold, ...
    'score_basis', 'dimension_separated_matched_truth_measurement_coverage', ...
    'min_labeled_assoc', a2.min_labeled_assoc, ...
    'n_output_tracks', a2.n_output_tracks + a3.n_output_tracks, ...
    'n_truth_reference', a2.n_truth_reference + a3.n_truth_reference, ...
    'n_evaluable_tracks', a2.n_evaluable_tracks + a3.n_evaluable_tracks, ...
    'n_correct_tracks', a2.n_correct_tracks + a3.n_correct_tracks, ...
    'n_error_or_extra_tracks', a2.n_error_or_extra_tracks + a3.n_error_or_extra_tracks, ...
    'track_ids', ids, 'purity', coverage, 'coverage', coverage, ...
    'coverage_distribution', summarize_track_coverage(coverage), ...
    'association_consistency', consistency, 'is_correct', correct, ...
    'accuracy_vs_output', safe_ratio(nnz(correct), numel(ids)), ...
    'accuracy_vs_truth', safe_ratio(nnz(correct), ...
        a2.n_truth_reference + a3.n_truth_reference));
end

function result = combine_start_time(a2, a3)
result = struct('n_truth', a2.n_truth + a3.n_truth, ...
    'n_started', a2.n_started + a3.n_started, ...
    'n_main_confirmed', a2.n_main_confirmed + a3.n_main_confirmed, ...
    'n_fragment_candidates', a2.n_fragment_candidates + a3.n_fragment_candidates, ...
    'mean_track_start_delay_s', weighted_mean( ...
        [a2.mean_track_start_delay_s, a3.mean_track_start_delay_s], ...
        [a2.n_started, a3.n_started]), ...
    'mean_main_track_start_delay_s', weighted_mean( ...
        [a2.mean_main_track_start_delay_s, a3.mean_main_track_start_delay_s], ...
        [a2.n_main_confirmed, a3.n_main_confirmed]));
end

function result = combine_angle_metrics(a2, a3)
result = angle_metrics([], [], []);
result.n = a2.n + a3.n;
if result.n == 0, return; end
result.rmse_az_deg = weighted_rmse(a2.rmse_az_deg, a2.n, a3.rmse_az_deg, a3.n);
result.rmse_el_deg = weighted_rmse(a2.rmse_el_deg, a2.n, a3.rmse_el_deg, a3.n);
result.rmse_los_deg = weighted_rmse(a2.rmse_los_deg, a2.n, a3.rmse_los_deg, a3.n);
end

function result = combine_position_metrics(p2, p3)
result = position_metrics(zeros(3, 0));
result.n = p2.n + p3.n;
if result.n == 0, return; end
result.rmse_e_m = weighted_rmse(p2.rmse_e_m, p2.n, p3.rmse_e_m, p3.n);
result.rmse_n_m = weighted_rmse(p2.rmse_n_m, p2.n, p3.rmse_n_m, p3.n);
result.rmse_u_m = weighted_rmse(p2.rmse_u_m, p2.n, p3.rmse_u_m, p3.n);
result.rmse_3d_m = weighted_rmse(p2.rmse_3d_m, p2.n, p3.rmse_3d_m, p3.n);
end

function result = combine_output_coverage(c2, c3)
n_reference = c2.n_reference_measurements + c3.n_reference_measurements;
n_covered = c2.n_covered_measurements + c3.n_covered_measurements;
result = struct('basis', 'dimension_separated_same_event_formal_output', ...
    'n_reference_measurements', n_reference, ...
    'n_covered_measurements', n_covered, ...
    'n_uncovered_measurements', n_reference - n_covered, ...
    'rate', safe_ratio(n_covered, n_reference), ...
    'n_matched_tracks', c2.n_matched_tracks + c3.n_matched_tracks, ...
    'per_track', [reshape(c3.per_track, [], 1); reshape(c2.per_track, [], 1)]);
end

function value = weighted_mean(values, weights)
keep = isfinite(values) & isfinite(weights) & weights > 0;
if ~any(keep), value = NaN; return; end
value = sum(values(keep) .* weights(keep)) / sum(weights(keep));
end

function value = weighted_rmse(a, na, b, nb)
values = [a, b]; weights = [na, nb];
keep = isfinite(values) & isfinite(weights) & weights > 0;
if ~any(keep), value = NaN; return; end
value = sqrt(sum(values(keep).^2 .* weights(keep)) / sum(weights(keep)));
end

function m = association_metrics(n_meas, assigned_ids, confirmed_ids)
n_assigned = numel(assigned_ids);
n_confirmed = nnz(ismember(assigned_ids, confirmed_ids));
m = struct('basis', 'measurement_dimension', ...
    'n_measurements', n_meas, ...
    'n_assigned', n_assigned, ...
    'n_assigned_confirmed', n_confirmed, ...
    'rate_all_tracks', safe_ratio(n_assigned, n_meas), ...
    'rate_confirmed_tracks', safe_ratio(n_confirmed, n_meas), ...
    'n_mode_assigned', 0);
end

function m = measurement_accuracy_metrics(n_meas, n_correct)
m = struct( ...
    'basis', 'correct_associations_over_input_measurements', ...
    'n_measurements', n_meas, ...
    'n_correct', n_correct, ...
    'n_not_correct', max(n_meas - n_correct, 0), ...
    'rate', safe_ratio(n_correct, n_meas));
end

function m = association_consistency_metrics(n_assigned, n_correct)
m = struct( ...
    'basis', 'correct_associations_over_assigned_measurements', ...
    'n_assigned', n_assigned, ...
    'n_correct', n_correct, ...
    'n_not_correct', max(n_assigned - n_correct, 0), ...
    'rate', safe_ratio(n_correct, n_assigned));
end

function acc = id_accuracy_metrics(track_id, truth_id, output_ids, truth_reference, cfg)
valid = isfinite(track_id) & isfinite(truth_id);
track_id = track_id(valid);
truth_id = truth_id(valid);
truth_reference = truth_reference(isfinite(truth_reference));
[rows, ~, row_index] = unique(truth_id);
[cols, ~, col_index] = unique(track_id);
C = accumarray([row_index(:), col_index(:)], 1, ...
    [numel(rows), numel(cols)], @sum, 0);

pairs = count_pairs(C, rows, cols, truth_reference);
correct = sum([pairs.n_correct]);
if isempty(pairs), correct = 0; end
acc = struct();
acc.n_assoc_total = numel(valid);
acc.n_labeled_assoc = nnz(valid);
acc.n_correct = correct;
acc.n_error = numel(track_id) - correct;
acc.accuracy = safe_ratio(correct, numel(track_id));
acc.n_truth = numel(rows);
acc.n_tracks = numel(cols);
acc.truth_ids = rows;
acc.track_ids = cols;
acc.confusion = C;
acc.pairs = pairs;
acc.fragmented_truth_count = nnz(sum(C > 0, 2) > 1);
acc.mixed_track_count = nnz(sum(C > 0, 1) > 1);

output_ids = unique(output_ids(isfinite(output_ids)));
[~, output_cols] = ismember(output_ids, cols);
valid_output = output_cols > 0;
Cout = zeros(numel(rows), numel(output_ids));
Cout(:, valid_output) = C(:, output_cols(valid_output));
output_pairs = count_pairs(Cout, rows, output_ids, truth_reference);
acc.output_pairs = output_pairs;
acc.output_labeled_counts = sum(Cout, 1);
acc.track_level = track_accuracy_metrics(Cout, output_ids, ...
    output_pairs, truth_reference, cfg);
end

function m = track_accuracy_metrics(C, output_ids, pairs, truth_reference, cfg)
purity_th = get_cfg(cfg, 'track_accuracy_purity_th', 0.9);
min_assoc = max(1, round(get_cfg(cfg, 'track_accuracy_min_assoc', 3)));
n_output = numel(output_ids);
evaluable = false(1, n_output);
correct = false(1, n_output);
purity = nan(1, n_output);
consistency = nan(1, n_output);
for c = 1:n_output
    n_track_assoc = sum(C(:, c));
    evaluable(c) = n_track_assoc >= min_assoc;
    p = find([pairs.track_id] == output_ids(c), 1);
    if isempty(p), continue; end
    % The numerator and complete reference count use the same measurement
    % scope (RAE / passive AE entering 2-D / overall), never formal retention.
    purity(c) = pairs(p).coverage;
    consistency(c) = safe_ratio(pairs(p).n_correct, n_track_assoc);
    correct(c) = evaluable(c) && pairs(p).n_correct >= min_assoc && ...
        purity(c) >= purity_th;
end
n_truth_reference = numel(unique(truth_reference(isfinite(truth_reference))));
coverage_distribution = summarize_track_coverage(purity);
m = struct('purity_threshold', purity_th, ...
    'score_basis', 'matched_truth_measurement_coverage', ...
    'min_labeled_assoc', min_assoc, ...
    'n_output_tracks', n_output, 'n_truth_reference', n_truth_reference, ...
    'n_evaluable_tracks', nnz(evaluable), ...
    'n_correct_tracks', nnz(correct), ...
    'n_error_or_extra_tracks', n_output - nnz(correct), ...
    'track_ids', output_ids, 'purity', purity, 'coverage', purity, ...
    'coverage_distribution', coverage_distribution, ...
    'association_consistency', consistency, 'is_correct', correct, ...
    'accuracy_vs_output', safe_ratio(nnz(correct), n_output), ...
    'accuracy_vs_truth', safe_ratio(nnz(correct), n_truth_reference));
end

function d = summarize_track_coverage(coverage)
% Distribution over matched output tracks only. Unmatched/extra tracks remain
% visible in n_error_or_extra_tracks and accuracy_vs_output.
values = coverage(isfinite(coverage));
d = struct('basis', 'matched_output_tracks', 'n_tracks', numel(values), ...
    'mean', NaN, 'median', NaN, ...
    'n_ge_90', 0, 'n_ge_95', 0, 'n_ge_98', 0, 'n_ge_99', 0, ...
    'rate_ge_90', NaN, 'rate_ge_95', NaN, ...
    'rate_ge_98', NaN, 'rate_ge_99', NaN);
if isempty(values), return; end
d.mean = mean(values);
d.median = median(values);
d.n_ge_90 = nnz(values >= 0.90);
d.n_ge_95 = nnz(values >= 0.95);
d.n_ge_98 = nnz(values >= 0.98);
d.n_ge_99 = nnz(values >= 0.99);
d.rate_ge_90 = d.n_ge_90 / d.n_tracks;
d.rate_ge_95 = d.n_ge_95 / d.n_tracks;
d.rate_ge_98 = d.n_ge_98 / d.n_tracks;
d.rate_ge_99 = d.n_ge_99 / d.n_tracks;
end

function pairs = count_pairs(C, truth_ids, track_ids, truth_reference)
pairs = repmat(struct('truth_id', NaN, 'track_id', NaN, 'n_correct', 0, ...
    'truth_assoc', 0, 'track_assoc', 0, 'truth_total', 0, 'coverage', NaN, ...
    'association_consistency', NaN), 0, 1);
if isempty(C) || ~any(C(:) > 0), return; end
max_count = max(C(:));
ij = solve_global_assignment(max_count - C, max_count);
for q = 1:size(ij, 1)
    r = ij(q, 1); c = ij(q, 2);
    if C(r, c) <= 0, continue; end
    p = struct();
    p.truth_id = truth_ids(r);
    p.track_id = track_ids(c);
    p.n_correct = C(r, c);
    p.truth_assoc = sum(C(r, :));
    p.track_assoc = sum(C(:, c));
    p.truth_total = nnz(truth_reference == truth_ids(r));
    p.coverage = safe_ratio(C(r, c), p.truth_total);
    p.association_consistency = safe_ratio(C(r, c), p.track_assoc);
    pairs(end + 1, 1) = p; %#ok<AGROW>
end
end

function s = start_delay_metrics(acc, truth_id, truth_t, output_id, output_t, ...
        assoc_track, assoc_truth, assoc_t)
valid_truth = isfinite(truth_id) & isfinite(truth_t);
truth_id = truth_id(valid_truth);
truth_t = truth_t(valid_truth);
truth_keys = unique(truth_id);
delays = zeros(1, 0);
main_delays = zeros(1, 0);
n_started = 0;
n_main = 0;
n_fragment_candidates = 0;
for q = 1:numel(truth_keys)
    tid = truth_keys(q);
    t0 = min(truth_t(truth_id == tid));
    r = find(acc.truth_ids == tid, 1);
    candidate_ids = zeros(1, 0);
    if ~isempty(r) && ~isempty(acc.confusion)
        candidate_ids = acc.track_ids(acc.confusion(r, :) > 0);
        candidate_ids = intersect(candidate_ids, unique(output_id), 'stable');
    end
    n_fragment_candidates = n_fragment_candidates + numel(candidate_ids);
    first_t = inf;
    for c = 1:numel(candidate_ids)
        first_correct_assoc = min_or_inf(assoc_t( ...
            assoc_track == candidate_ids(c) & assoc_truth == tid));
        if ~isfinite(first_correct_assoc), continue; end
        tt = output_t(output_id == candidate_ids(c));
        tt = tt(isfinite(tt) & tt >= max(t0, first_correct_assoc));
        if ~isempty(tt), first_t = min(first_t, min(tt)); end
    end
    if isfinite(first_t)
        n_started = n_started + 1;
        delays(end + 1) = first_t - t0; %#ok<AGROW>
    end

    p = find([acc.output_pairs.truth_id] == tid, 1);
    if ~isempty(p)
        main_track = acc.output_pairs(p).track_id;
        first_correct_assoc = min_or_inf(assoc_t( ...
            assoc_track == main_track & assoc_truth == tid));
        tt = output_t(output_id == main_track);
        tt = tt(isfinite(tt) & tt >= max(t0, first_correct_assoc));
        if ~isempty(tt)
            n_main = n_main + 1;
            main_delays(end + 1) = min(tt) - t0; %#ok<AGROW>
        end
    end
end
s = struct('n_truth', numel(truth_keys), 'n_started', n_started, ...
    'n_main_confirmed', n_main, 'n_fragment_candidates', n_fragment_candidates, ...
    'mean_track_start_delay_s', mean_or_nan(delays), ...
    'mean_main_track_start_delay_s', mean_or_nan(main_delays));
end

function value = min_or_inf(values)
values = values(isfinite(values));
if isempty(values), value = inf; else, value = min(values); end
end

function [angle, position] = output_error_metrics(data, dim, output_mask, pairs)
[az_err, el_err, pos_err, ~, ~, los_err] = output_error_samples(data, dim, output_mask, pairs);
angle = angle_metrics(az_err, el_err, los_err);
position = position_metrics(pos_err);
end

function [az_err, el_err, pos_err, angle_track, position_track, los_err] = ...
        output_error_samples(data, dim, output_mask, pairs)
az_err = zeros(1, 0); el_err = zeros(1, 0); pos_err = zeros(3, 0);
los_err = zeros(1, 0);
angle_track = zeros(1, 0); position_track = zeros(1, 0);
out_index = find(output_mask);
if isempty(out_index) || isempty(pairs)
    return;
end
pair_track = reshape([pairs.track_id], 1, []);
pair_truth = reshape([pairs.truth_id], 1, []);
[matched, pair_index] = ismember(data.output_track(out_index), pair_track);
out_index = out_index(matched);
paired_truth = pair_truth(pair_index(matched));

[angle_keys, ref_az, ref_el] = angle_reference_table(data, dim);
if ~isempty(out_index) && ~isempty(angle_keys)
    query = [data.output_event(out_index).', paired_truth(:)];
    [has_reference, reference_index] = ismember(query, angle_keys, 'rows');
    valid = has_reference & isfinite(data.output_az(out_index)).' & ...
        isfinite(data.output_el(out_index)).';
    candidate = find(valid);
    if ~isempty(candidate)
        r = reference_index(candidate);
        finite_reference = isfinite(ref_az(r)) & isfinite(ref_el(r));
        candidate = candidate(finite_reference); r = r(finite_reference);
        az_err = angle_diff(data.output_az(out_index(candidate)), ref_az(r).');
        el_err = data.output_el(out_index(candidate)) - ref_el(r).';
        los_err = los_separation_deg(data.output_az(out_index(candidate)), ...
            data.output_el(out_index(candidate)), ref_az(r).', ref_el(r).');
        angle_track = data.output_track(out_index(candidate));
    end
end

position_candidate = data.output_dim(out_index) == 3 & ...
    all(isfinite(data.output_pos(:, out_index)), 1);
if any(position_candidate)
    position_index = out_index(position_candidate);
    position_truth = paired_truth(position_candidate);
    [position_keys, ref_position] = position_reference_table(data);
    query = [data.output_event(position_index).', position_truth(:)];
    [has_reference, reference_index] = ismember(query, position_keys, 'rows');
    candidate = find(has_reference);
    if ~isempty(candidate)
        r = reference_index(candidate);
        finite_reference = all(isfinite(ref_position(:, r)), 1);
        candidate = candidate(finite_reference); r = r(finite_reference);
        pos_err = data.output_pos(:, position_index(candidate)) - ref_position(:, r);
        position_track = data.output_track(position_index(candidate));
    end
end
end

function coverage = formal_output_coverage(data, dim, output_mask, pairs, output_ids)
[truth_id, truth_event] = evaluation_truth_scope(data, dim);
per_track = repmat(struct('track_id', NaN, 'truth_id', NaN, ...
    'n_reference_measurements', 0, 'n_covered_measurements', 0, ...
    'rate', NaN), numel(output_ids), 1);
valid_reference = isfinite(truth_id) & isfinite(truth_event);
covered_reference = false(size(truth_id));
pair_track = reshape([pairs.track_id], 1, []);
pair_truth = reshape([pairs.truth_id], 1, []);
[matched_tracks, pair_index] = ismember(output_ids, pair_track);
track_truth = nan(size(output_ids));
track_truth(matched_tracks) = pair_truth(pair_index(matched_tracks));

out_index = find(output_mask);
if ~isempty(out_index) && ~isempty(pair_track)
    [matched_output, output_pair_index] = ...
        ismember(data.output_track(out_index), pair_track);
    formal_keys = [data.output_event(out_index(matched_output)).', ...
        pair_truth(output_pair_index(matched_output)).'];
    formal_keys = unique(formal_keys(all(isfinite(formal_keys), 2), :), 'rows');
    reference_query = [truth_event(valid_reference).', truth_id(valid_reference).'];
    covered_reference(valid_reference) = ismember(reference_query, formal_keys, 'rows').';
end

reference_truth = truth_id(valid_reference);
covered_truth = truth_id(valid_reference & covered_reference);
truth_keys = unique(reference_truth);
reference_counts = zeros(size(truth_keys)); covered_counts = zeros(size(truth_keys));
if ~isempty(truth_keys)
    [~, location] = ismember(reference_truth, truth_keys);
    reference_counts = accumarray(location(:), 1, [numel(truth_keys), 1]).';
    if ~isempty(covered_truth)
        [~, location] = ismember(covered_truth, truth_keys);
        covered_counts = accumarray(location(:), 1, [numel(truth_keys), 1]).';
    end
end
for q = 1:numel(output_ids)
    id = output_ids(q);
    per_track(q).track_id = id;
    if ~matched_tracks(q), continue; end
    tid = track_truth(q);
    [found, location] = ismember(tid, truth_keys);
    if found
        n_ref = reference_counts(location); n_covered_track = covered_counts(location);
    else
        n_ref = 0; n_covered_track = 0;
    end
    per_track(q).truth_id = tid;
    per_track(q).n_reference_measurements = n_ref;
    per_track(q).n_covered_measurements = n_covered_track;
    per_track(q).rate = safe_ratio(n_covered_track, n_ref);
end
n_reference = nnz(valid_reference);
n_covered = nnz(covered_reference & valid_reference);
coverage = struct('basis', 'same_event_formal_output', ...
    'n_reference_measurements', n_reference, ...
    'n_covered_measurements', n_covered, ...
    'n_uncovered_measurements', n_reference - n_covered, ...
    'rate', safe_ratio(n_covered, n_reference), ...
    'n_matched_tracks', nnz(matched_tracks), 'per_track', per_track);
end

function [truth_id, truth_event, truth_t, basis] = evaluation_truth_scope(data, dim)
if dim == 2
    truth_id = data.truth_2d_id;
    truth_event = data.truth_2d_event;
    truth_t = data.truth_2d_t;
    basis = 'all_input_ae_physical_passive_and_active_ae_only';
elseif dim == 3
    truth_id = data.truth_3d_id;
    truth_event = data.truth_3d_event;
    truth_t = data.truth_3d_t;
    basis = 'all_input_rae';
else
    truth_id = [data.truth_2d_id, data.truth_3d_id];
    truth_event = [data.truth_2d_event, data.truth_3d_event];
    truth_t = [data.truth_2d_t, data.truth_3d_t];
    basis = 'all_input_measurements';
end
end

function [keys, ref_az, ref_el] = angle_reference_table(data, dim)
[truth_id, truth_event, truth_az, truth_el] = truth_scope_arrays(data, dim);
valid = isfinite(truth_id) & isfinite(truth_event) & ...
    isfinite(truth_az) & isfinite(truth_el);
if ~any(valid)
    keys = zeros(0, 2); ref_az = zeros(0, 1); ref_el = zeros(0, 1);
    return;
end
[keys, ~, group] = unique([truth_event(valid).', truth_id(valid).'], 'rows');
n = size(keys, 1); count = accumarray(group, 1, [n, 1]);
sin_sum = accumarray(group, reshape(sind(truth_az(valid)), [], 1), [n, 1], @sum);
cos_sum = accumarray(group, reshape(cosd(truth_az(valid)), [], 1), [n, 1], @sum);
el_sum = accumarray(group, reshape(truth_el(valid), [], 1), [n, 1], @sum);
ref_az = atan2d(sin_sum ./ count, cos_sum ./ count);
ref_el = el_sum ./ count;
end

function [keys, ref_position] = position_reference_table(data)
valid = isfinite(data.truth_3d_id) & isfinite(data.truth_3d_event) & ...
    all(isfinite(data.truth_3d_xyz), 1);
if ~any(valid)
    keys = zeros(0, 2); ref_position = zeros(3, 0);
    return;
end
[keys, ~, group] = unique( ...
    [data.truth_3d_event(valid).', data.truth_3d_id(valid).'], 'rows');
n = size(keys, 1); count = accumarray(group, 1, [n, 1]);
ref_position = zeros(3, n);
for axis = 1:3
    total = accumarray(group, reshape(data.truth_3d_xyz(axis, valid), [], 1), ...
        [n, 1], @sum);
    ref_position(axis, :) = (total ./ count).';
end
end

function [id, event, az, el] = truth_scope_arrays(data, dim)
if dim == 2
    id = data.truth_2d_id; event = data.truth_2d_event;
    az = data.truth_2d_az; el = data.truth_2d_el;
elseif dim == 3
    id = data.truth_3d_id; event = data.truth_3d_event;
    az = data.truth_3d_az; el = data.truth_3d_el;
else
    id = [data.truth_2d_id, data.truth_3d_id];
    event = [data.truth_2d_event, data.truth_3d_event];
    az = [data.truth_2d_az, data.truth_3d_az];
    el = [data.truth_2d_el, data.truth_3d_el];
end
end

function m = angle_metrics(az_err, el_err, los_err)
m = struct('n', numel(az_err), 'rmse_az_deg', NaN, ...
    'rmse_el_deg', NaN, 'rmse_los_deg', NaN);
if isempty(az_err), return; end
m.rmse_az_deg = sqrt(mean(az_err.^2));
m.rmse_el_deg = sqrt(mean(el_err.^2));
m.rmse_los_deg = sqrt(mean(los_err.^2));
end

function m = position_metrics(pos_err)
m = struct('n', size(pos_err, 2), 'rmse_e_m', NaN, ...
    'rmse_n_m', NaN, 'rmse_u_m', NaN, 'rmse_3d_m', NaN);
if isempty(pos_err), return; end
m.rmse_e_m = sqrt(mean(pos_err(1, :).^2));
m.rmse_n_m = sqrt(mean(pos_err(2, :).^2));
m.rmse_u_m = sqrt(mean(pos_err(3, :).^2));
m.rmse_3d_m = sqrt(mean(sum(pos_err.^2, 1)));
end

function print_joint_report(metrics)
fprintf('\n========== 二维/三维分维度定量评价 ==========\n');
fprintf('\n[量测伪真值一致性]\n');
T = metrics.truth_targets;
fprintf('量测标签参考数: 原始编号=%d, 拆分后实例=%d, 被拆分原始编号=%d\n', ...
    T.raw_id_count, T.instance_count, T.n_split_raw_ids);
if get_cfg(T, 'passive_truth_split_enabled', false)
    fprintf('  纯被动时间拆分: 开启, 相邻AE间隔>%.3fs时拆分, 被拆分编号=%d\n', ...
        get_cfg(T, 'passive_max_gap_s', NaN), ...
        get_cfg(T, 'n_passive_time_split_raw_ids', 0));
end
if T.enabled && T.n_angle_only_raw_ids > 0
    fprintf('  其中纯角度编号=%d（无三维位置，实例数服从主动/纯被动拆分配置）\n', ...
        T.n_angle_only_raw_ids);
end
print_scope_report('二维角度航迹/输出（物理被动AE + 主动AE-only）', metrics.two_d, 2);
print_scope_report('三维主动空间', metrics.three_d, 3);
print_scope_report('二维/三维分域汇总（不跨维重新匹配）', metrics.overall, 0);
print_output_identity_audit(metrics.output_identity_audit);
F = metrics.measurement_flow;
fprintf('\n[物理量测维度 -> 关联航迹维度]\n');
fprintf('  二维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(1, :));
fprintf('  三维量测: 关联后未保留=%d, 二维=%d, 三维=%d\n', F.counts(2, :));
fprintf('  主动距离关联: %d, 完成空间更新=%d, 未更新=%d\n', ...
    F.n_range_associated, F.n_range_updated, ...
    F.n_range_associated_without_update);
fprintf('\n[量测来源 -> 去向（未分配/关联未保留/二维/三维）]\n');
fprintf('  主动RAE:       %d / %d / %d / %d\n', F.source_counts(1, :));
fprintf('  主动AE-only:   %d / %d / %d / %d\n', F.source_counts(2, :));
fprintf('  物理被动AE:    %d / %d / %d / %d\n', F.source_counts(3, :));
A = metrics.measurement_accounting;
fprintf('\n[物理量测去向]\n');
print_accounting_row('主动全部', A.active);
print_accounting_row('主动距离', A.active_range);
print_accounting_row('主动AE-only', A.active_ae_only);
print_accounting_row('物理被动AE', A.physical_passive_ae);
end

function print_output_identity_audit(audit)
if ~isstruct(audit) || ~isfield(audit, 'status') || ...
        ~strcmp(audit.status, 'ok')
    return;
end
fprintf('\n[三维公开输出ID对账]\n');
fprintf(['  成熟external=%d；联合branch=%d；公开ID=%d；' ...
    '绘图(>=%d点)=%d；内部owner别名折叠=%d。\n'], ...
    audit.n_mature_3d_external_ids, audit.n_joint_3d_branch_ids, ...
    audit.n_public_3d_ids, audit.plot_min_life, ...
    audit.n_plot_visible_3d_ids, audit.n_internal_owner_aliases_collapsed);
rows = audit.three_d_rows;
if isempty(rows), return; end
special = ~[rows.is_mature_external] | ~[rows.is_plot_visible] | ...
    [rows.n_internal_logical_ids] > 1;
rows = rows(special);
if isempty(rows)
    fprintf('  差异明细: 无；三套集合完全一致。\n');
    return;
end
fprintf(['  差异明细（公开ID branch3D internalLogicalIDs 3D点数 ' ...
    '绘图 成熟来源 匹配真值 覆盖率）:\n']);
for q = 1:numel(rows)
    fprintf('    %g  %g  %s  %d  %d  %d  %.15g  %.2f%%\n', ...
        rows(q).public_id, rows(q).branch_3d_id, ...
        mat2str(rows(q).internal_logical_ids), rows(q).n_3d_output_points, ...
        rows(q).is_plot_visible, rows(q).is_mature_external, ...
        rows(q).matched_truth_id, 100 * rows(q).truth_coverage);
end
if ~isempty(audit.mature_without_joint_3d_ids)
    fprintf('  成熟主干有正式输出、联合层没有三维正式输出的external ID: %s\n', ...
        mat2str(audit.mature_without_joint_3d_ids));
end
end

function print_accounting_row(label, row)
fprintf(['  %s: 输入=%d, 量测利用(关联或新生)=%d(%.2f%%), ' ...
    '去2D=%d, 去3D=%d, 关联/新生后未保留=%d, ' ...
    '未关联=%d, 未解释=%d, 去向字段差=%d\n'], ...
    label, row.n_input, row.n_associated_or_born, 100 * row.utilization_rate, ...
    row.n_to_2d, row.n_to_3d, row.n_associated_not_retained, ...
    row.n_unassigned, row.unaccounted, row.destination_gap);
end

function print_scope_report(label, s, dim)
fprintf('\n[%s]\n', label);
if dim == 2
    fprintf('  输出: %d点, %d个二维公开ID\n', ...
        s.output.n_outputs, s.output.n_unique_tracks);
    fprintf('  二维输入总数=%d（物理被动AE=%d, 主动AE-only=%d）, 带标签参考=%d\n', ...
        s.association.n_measurements, s.reference.n_passive_input, ...
        s.reference.n_active_ae_only_input, s.reference.n_labeled_measurements);
    fprintf('  被动输入分流: 进入2D=%d, 进入3D=%d, 未送入支路=%d\n', ...
        s.reference.n_passive_to_2d, s.reference.n_passive_to_3d, ...
        s.reference.n_passive_not_routed);
    fprintf('  主动AE-only分流: 进入2D=%d, 进入3D=%d, 未送入支路=%d（不计三维距离/位置得分）\n', ...
        s.reference.n_active_ae_only_to_2d, ...
        s.reference.n_active_ae_only_to_3d, ...
        s.reference.n_active_ae_only_not_routed);
elseif dim == 3
    fprintf('  输出: %d点, %d个三维公开ID\n', ...
        s.output.n_outputs, s.output.n_unique_tracks);
    fprintf('  三维带标签RAE参考=%d（被动AE不参与三维得分计数）\n', ...
        s.reference.n_labeled_measurements);
else
    fprintf(['  输出: %d点, %d个分维公开ID（二维%d + 三维%d；' ...
        '不是物理目标数）\n'], s.output.n_outputs, s.output.n_unique_tracks, ...
        s.output.n_2d_tracks, s.output.n_3d_tracks);
end
fprintf('  量测关联利用率(全部/曾确认): %.2f%% / %.2f%%  (%d/%d, %d/%d)\n', ...
    100 * s.association.rate_all_tracks, 100 * s.association.rate_confirmed_tracks, ...
    s.association.n_assigned, s.association.n_measurements, ...
    s.association.n_assigned_confirmed, s.association.n_measurements);
fprintf('  量测正确率: %.2f%%  正确=%d, 二维/三维输入=%d, 非正确关联=%d\n', ...
    100 * s.measurement_accuracy.rate, s.measurement_accuracy.n_correct, ...
    s.measurement_accuracy.n_measurements, ...
    s.measurement_accuracy.n_not_correct);
fprintf('  已关联一致率: %.2f%%  正确=%d, 已关联=%d, 非正确关联=%d\n', ...
    100 * s.association_consistency.rate, ...
    s.association_consistency.n_correct, ...
    s.association_consistency.n_assigned, ...
    s.association_consistency.n_not_correct);
fprintf('  航迹级正确率(比输出/比参考): %.2f%% / %.2f%%  正确航迹数=%d, 输出=%d, 参考=%d\n', ...
    100 * s.track_accuracy.accuracy_vs_output, ...
    100 * s.track_accuracy.accuracy_vs_truth, ...
    s.track_accuracy.n_correct_tracks, s.track_accuracy.n_output_tracks, ...
    s.track_accuracy.n_truth_reference);
d = s.track_accuracy.coverage_distribution;
fprintf(['  航迹覆盖率分布(已匹配输出=%d): 均值=%.2f%%, 中位数=%.2f%%; ' ...
    '>=90/95/98/99%%: %.2f/%.2f/%.2f/%.2f%%\n'], ...
    d.n_tracks, 100 * d.mean, 100 * d.median, ...
    100 * d.rate_ge_90, 100 * d.rate_ge_95, ...
    100 * d.rate_ge_98, 100 * d.rate_ge_99);
fprintf('  正式输出同步覆盖率: %.2f%%  覆盖=%d/%d个带标签物理量测点\n', ...
    100 * s.output_coverage.rate, s.output_coverage.n_covered_measurements, ...
    s.output_coverage.n_reference_measurements);
fprintf('  航迹起始: 最早确认=%d/%d, 平均延迟=%.3fs; 主航迹=%d/%d, 平均延迟=%.3fs\n', ...
    s.start_time.n_started, s.start_time.n_truth, ...
    s.start_time.mean_track_start_delay_s, s.start_time.n_main_confirmed, ...
    s.start_time.n_truth, s.start_time.mean_main_track_start_delay_s);
if dim ~= 0 && s.angle.n > 0
    if dim == 3
        angle_label = '三维航迹角度投影RMSE';
    elseif dim == 2
        angle_label = '二维角度RMSE';
    else
        angle_label = '全部输出角度RMSE';
    end
    fprintf('  %s: az=%.4fdeg, el=%.4fdeg, LOS=%.4fdeg (%d点)\n', ...
        angle_label, s.angle.rmse_az_deg, s.angle.rmse_el_deg, ...
        s.angle.rmse_los_deg, s.angle.n);
end
if dim ~= 2 && s.position.n > 0
    fprintf('  三维位置RMSE: E/N/U=[%.2f %.2f %.2f]m, 3D=%.2fm (%d点)\n', ...
        s.position.rmse_e_m, s.position.rmse_n_m, s.position.rmse_u_m, ...
        s.position.rmse_3d_m, s.position.n);
end
end

function a = event_assoc(est, k)
a = struct('id', zeros(1, 0), 'type', {cell(1, 0)}, ...
    'meas_index', zeros(1, 0), 'tid', zeros(1, 0));
if isfield(est, 'assoc') && k <= numel(est.assoc) && ~isempty(est.assoc{k})
    a = est.assoc{k};
end
end

function out = event_output(est, k)
out = repmat(struct('id', 0, 'truth_id', NaN, 'output_dim', 0, ...
    't_sec', NaN, 'az_deg', NaN, 'el_deg', NaN, ...
    'position_enu', nan(3, 1)), 0, 1);
if isfield(est, 'output') && k <= numel(est.output) && ~isempty(est.output{k})
    out = est.output{k};
elseif isfield(est, 'output_history')
    out = joint_review_output('event', est.output_history, k);
end
end

function dim = association_measurement_dim(a, q, e)
dim = 0;
if isfield(a, 'measurement_dim') && q <= numel(a.measurement_dim) && ...
        isfinite(a.measurement_dim(q)) && any(a.measurement_dim(q) == [2, 3])
    dim = a.measurement_dim(q);
    return;
end

type = indexed_text(a, 'type', q, '');
if isempty(type), return; end
if association_type_matches(type, 'passive')
    dim = 2;
elseif association_type_matches(type, 'active')
    mi = indexed_field_value(a, 'meas_index', q, 0);
    if mi >= 1 && mi <= numel(e.active.has_range) && e.active.has_range(mi)
        dim = 3;
    else
        dim = 2;
    end
end
end

function source = association_source_kind(a, q, e, measurement_dim)
% 1=active RAE, 2=active AE-only, 3=physical passive AE.
source = 0;
type = indexed_text(a, 'type', q, '');
mi = round(indexed_field_value(a, 'meas_index', q, 0));
if association_type_matches(type, 'active')
    if measurement_dim == 3, source = 1; else, source = 2; end
elseif association_type_matches(type, 'passive')
    source = 3;
    if mi >= 1 && isfield(e.passive, 'kind') && mi <= numel(e.passive.kind) && ...
            e.passive.kind(mi) == 2
        source = 2;
    end
end
end

function source = passive_measurement_sources(passive, n)
source = 3 * ones(1, n);
if n == 0 || ~isfield(passive, 'kind') || isempty(passive.kind), return; end
kind = sized_row(passive.kind, n, 1);
source(kind == 2) = 2;
end

function truth = association_truth_label(a, q, ~, labels, k)
truth = NaN;
if isfield(a, 'tid'), truth = indexed_value(a.tid, q, NaN); end
type = indexed_text(a, 'type', q, '');
mi = round(indexed_field_value(a, 'meas_index', q, 0));
if mi < 1, return; end
if association_type_matches(type, 'active') && k <= numel(labels.active) && ...
        mi <= numel(labels.active{k})
    truth = labels.active{k}(mi);
elseif association_type_matches(type, 'passive') && ...
        k <= numel(labels.passive) && ...
        mi <= numel(labels.passive{k})
    truth = labels.passive{k}(mi);
end
end

function dims = association_filter_dimensions(a, est, k)
ids = reshape(a.id, 1, []);
dims = zeros(size(ids));
if isempty(ids), return; end
if isfield(a, 'filter_dim') && ~isempty(a.filter_dim)
    n = min(numel(ids), numel(a.filter_dim));
    supplied = reshape(a.filter_dim(1:n), 1, []);
    valid = isfinite(supplied) & ismember(supplied, [2, 3]);
    dims(find(valid)) = supplied(valid); %#ok<FNDSB>
end
unresolved = dims == 0;
out = event_output(est, k);
if any(unresolved) && ~isempty(out)
    out_ids = reshape([out.id], 1, []);
    out_dims = reshape([out.output_dim], 1, []);
    [found, location] = ismember(ids(unresolved), out_ids);
    target = find(unresolved); target = target(found);
    dims(target) = out_dims(location(found));
end
unresolved = dims == 0;
if ~any(unresolved) || ~isfield(est, 'logical_tracks') || ...
        k > numel(est.logical_tracks) || isempty(est.logical_tracks{k})
    return;
end
tracks = est.logical_tracks{k}; track_ids = reshape([tracks.id], 1, []);
[found, location] = ismember(ids(unresolved), track_ids);
target = find(unresolved); target = target(found);
matched_location = location(found);
for q = 1:numel(target)
    dims(target(q)) = snapshot_dimension(tracks(matched_location(q)));
end
end

function dims = association_input_dimensions(a, retained_dims)
dims = retained_dims;
if isfield(a, 'input_dim') && numel(a.input_dim) == numel(dims)
    dims = reshape(a.input_dim, 1, []);
else
    % A legacy passive birth with no retained track still entered the 2-D branch.
    for q = find(dims == 0)
        if strcmp(indexed_text(a, 'type', q, ''), 'passive_birth'), dims(q) = 2; end
    end
end
end

function dims = measurement_input_dimensions(est, a, assoc_dims, k, type, n)
dims = zeros(1, n);
for q = 1:numel(a.id)
    if ~association_type_matches(indexed_text(a, 'type', q, ''), type), continue; end
    mi = indexed_field_value(a, 'meas_index', q, 0);
    if isfinite(mi) && mi >= 1 && mi <= n && mi == round(mi)
        dims(mi) = assoc_dims(q);
    end
end
if isfield(est, 'measurement_disposition') && k <= numel(est.measurement_disposition)
    record = est.measurement_disposition{k};
    if isstruct(record) && isfield(record, type) && isfield(record.(type), 'input_dim')
        supplied = record.(type).input_dim;
        assert(numel(supplied) == n && all(ismember(supplied, [0, 2, 3])), ...
            'evaluate_joint_tracking_metrics:InvalidInputDimension', ...
            'Input branch ledger must match the event measurement indices.');
        dims = double(reshape(supplied, 1, []));
    end
end
end

function tf = association_type_matches(value, requested)
if isa(value, 'string') && isscalar(value), value = char(value); end
tf = ischar(value) && ~isempty(strfind(value, requested)); %#ok<STREMP>
end

function flow = build_measurement_flow(data)
filter_dims = [0, 2, 3];
M = zeros(2, numel(filter_dims));
for r = 1:2
    meas_dim = r + 1;
    for c = 1:numel(filter_dims)
        M(r, c) = nnz(data.assoc_meas_dim == meas_dim & ...
            data.assoc_filter_dim == filter_dims(c));
    end
end
range_mask = data.assoc_meas_dim == 3;
source_totals = [data.n_active_range_measurements, ...
    data.n_active_ae_only_measurements, data.n_passive_measurements];
source_counts = zeros(3, 4);
for source = 1:3
    mask = data.assoc_source == source;
    source_counts(source, 1) = max(source_totals(source) - nnz(mask), 0);
    source_counts(source, 2) = nnz(mask & data.assoc_filter_dim == 0);
    source_counts(source, 3) = nnz(mask & data.assoc_filter_dim == 2);
    source_counts(source, 4) = nnz(mask & data.assoc_filter_dim == 3);
end
if data.n_active_measurements ~= ...
        data.n_active_range_measurements + data.n_active_ae_only_measurements || ...
        data.n_meas_2d ~= ...
        data.n_active_ae_only_measurements + data.n_passive_measurements
    error('evaluate_joint_tracking_metrics:MeasurementSourceAccounting', ...
        '主动RAE/主动AE-only/物理被动AE的输入数量与二维/三维总量不守恒。');
end
if any(sum(source_counts, 2).' ~= source_totals)
    error('evaluate_joint_tracking_metrics:MeasurementDestinationAccounting', ...
        '量测来源到未分配/二维/三维去向的数量不守恒。');
end
flow = struct('row_measurement_dims', [2, 3], ...
    'column_filter_dims', filter_dims, 'counts', M, ...
    'source_names', {{'active_rae', 'active_ae_only', 'passive_ae'}}, ...
    'source_destination_names', {{'unassigned', ...
        'associated_not_retained', 'to_2d', 'to_3d'}}, ...
    'source_counts', source_counts, ...
    'n_range_associated', nnz(range_mask), ...
    'n_range_updated', nnz(range_mask & data.assoc_range_updated), ...
    'n_range_associated_without_update', ...
        nnz(range_mask & ~data.assoc_range_updated));
end

function accounting = build_measurement_accounting(~, data)
accounting = struct();
accounting.active = source_accounting_row(data.n_active_measurements, ...
    ismember(data.assoc_source, [1, 2]), data.assoc_filter_dim);
accounting.passive = source_accounting_row(data.n_passive_measurements, ...
    data.assoc_source == 3, data.assoc_filter_dim);
accounting.active_range = source_accounting_row( ...
    data.n_active_range_measurements, data.assoc_source == 1, ...
    data.assoc_filter_dim);
accounting.active_ae_only = source_accounting_row( ...
    data.n_active_ae_only_measurements, data.assoc_source == 2, ...
    data.assoc_filter_dim);
accounting.physical_passive_ae = source_accounting_row( ...
    data.n_passive_measurements, data.assoc_source == 3, ...
    data.assoc_filter_dim);
accounting.physical_2d = source_accounting_row(data.n_meas_2d, ...
    ismember(data.assoc_source, [2, 3]), data.assoc_filter_dim);
accounting.conserved = accounting.active.unaccounted == 0 && ...
    accounting.passive.unaccounted == 0 && ...
    accounting.active_range.unaccounted == 0 && ...
    accounting.active_ae_only.unaccounted == 0 && ...
    accounting.physical_passive_ae.unaccounted == 0;
accounting.destinations_complete = accounting.active.destination_gap == 0 && ...
    accounting.passive.destination_gap == 0 && ...
    accounting.active_range.destination_gap == 0 && ...
    accounting.active_ae_only.destination_gap == 0 && ...
    accounting.physical_passive_ae.destination_gap == 0;
end

function row = source_accounting_row(total, mask, filter_dim)
associated = nnz(mask);
unassigned = max(total - associated, 0);
row = accounting_row(total, associated, unassigned);
row.n_unassigned = unassigned;
% Compatibility alias: older report consumers used this field for every
% measurement without an association, even when no explicit guard rejected it.
row.n_explicitly_suppressed = unassigned;
row = attach_destinations(row, mask, filter_dim);
end

function row = accounting_row(total, associated, suppressed)
row = struct('n_input', total, 'n_associated_or_born', associated, ...
    'utilization_rate', safe_ratio(associated, total), ...
    'n_explicitly_suppressed', suppressed, ...
    'unaccounted', total - associated - suppressed);
end

function row = attach_destinations(row, mask, filter_dim)
row.n_to_2d = nnz(mask & filter_dim == 2);
row.n_to_3d = nnz(mask & filter_dim == 3);
row.n_associated_not_retained = nnz(mask & filter_dim == 0);
row.destination_gap = row.n_associated_or_born - row.n_to_2d - ...
    row.n_to_3d - row.n_associated_not_retained;
end

function details = build_track_details(data, metrics)
ids = unique(data.output_track(isfinite(data.output_track)));
details = repmat(track_detail_template(), numel(ids), 1);
[~, output_group] = ismember(data.output_track, ids);
[~, assoc_group] = ismember(data.assoc_track, ids);
n_id = numel(ids);
n_output = grouped_count(output_group, output_group > 0, n_id);
n_output_2d = grouped_count(output_group, output_group > 0 & data.output_dim == 2, n_id);
n_output_3d = grouped_count(output_group, output_group > 0 & data.output_dim == 3, n_id);
if any(n_output_2d > 0 & n_output_3d > 0)
    error('evaluate_joint_tracking_metrics:MixedDimensionOutputId', ...
        '同一个公开输出ID同时出现在二维和三维输出中，违反分维ID契约。');
end
n_assoc = grouped_count(assoc_group, assoc_group > 0, n_id);
n_active_range = grouped_count(assoc_group, assoc_group > 0 & ...
    data.assoc_is_active & data.assoc_meas_dim == 3, n_id);
n_active_angle = grouped_count(assoc_group, assoc_group > 0 & ...
    data.assoc_is_active & data.assoc_meas_dim == 2, n_id);
n_passive = grouped_count(assoc_group, assoc_group > 0 & data.assoc_is_passive, n_id);
n_output_event = zeros(n_id, 1);
valid_event = output_group > 0 & isfinite(data.output_event);
if any(valid_event)
    event_pairs = unique([output_group(valid_event).', data.output_event(valid_event).'], 'rows');
    n_output_event = accumarray(event_pairs(:, 1), 1, [n_id, 1]);
end
start_time = nan(n_id, 1); end_time = nan(n_id, 1);
valid_time = output_group > 0 & isfinite(data.output_t);
if any(valid_time)
    group = output_group(valid_time).'; time = data.output_t(valid_time).';
    start_time = accumarray(group, time, [n_id, 1], @min, NaN);
    end_time = accumarray(group, time, [n_id, 1], @max, NaN);
end
[two_d_detail, three_d_detail, overall_detail] = deal( ...
    build_track_scope_details(metrics.two_d, ids), ...
    build_track_scope_details(metrics.three_d, ids), ...
    build_track_scope_details(metrics.overall, ids));
[az2, el2, ~, angle_track2, ~, los2] = output_error_samples( ...
    data, 2, data.output_dim == 2, metrics.two_d.accuracy.output_pairs);
[az3, el3, position_error, angle_track3, position_track, los3] = ...
    output_error_samples(data, 3, data.output_dim == 3, ...
    metrics.three_d.accuracy.output_pairs);
az_error = [az3, az2]; el_error = [el3, el2];
los_error = [los3, los2]; angle_track = [angle_track3, angle_track2];
[angle_by_track, position_by_track] = grouped_error_metrics( ...
    ids, angle_track, az_error, el_error, position_track, position_error, los_error);
for q = 1:numel(ids)
    d = track_detail_template(); d.track_id = ids(q);
    d.n_output_points = n_output(q); d.n_output_events = n_output_event(q);
    d.n_output_2d = n_output_2d(q); d.n_output_3d = n_output_3d(q);
    if isfinite(start_time(q))
        d.start_time_s = start_time(q); d.end_time_s = end_time(q);
        d.duration_s = d.end_time_s - d.start_time_s;
    end
    d.n_associations = n_assoc(q); d.n_active_range_assoc = n_active_range(q);
    d.n_active_angle_assoc = n_active_angle(q); d.n_passive_assoc = n_passive(q);
    d.two_d = two_d_detail(q); d.three_d = three_d_detail(q);
    d.overall = overall_detail(q);
    if d.n_output_3d > 0
        d.primary_scope = 'three_d'; d.primary = d.three_d;
    else
        d.primary_scope = 'two_d'; d.primary = d.two_d;
    end
    a = angle_by_track(q); p = position_by_track(q);
    d.angle_rmse = struct('n', a.n, 'az_deg', a.rmse_az_deg, ...
        'el_deg', a.rmse_el_deg, 'los_deg', a.rmse_los_deg);
    d.position_rmse = struct('n', p.n, 'e_m', p.rmse_e_m, ...
        'n_m', p.rmse_n_m, 'u_m', p.rmse_u_m, ...
        'three_d_m', p.rmse_3d_m);
    details(q) = d;
end
end

function detail = build_track_scope_details(scope, ids)
n_id = numel(ids); detail = repmat(track_scope_detail_template(), n_id, 1);
[applicable, scope_index] = ismember(ids, scope.track_accuracy.track_ids);
pairs = scope.accuracy.output_pairs;
pair_track = reshape([pairs.track_id], 1, []);
[has_pair, pair_index] = ismember(ids, pair_track);
coverage = scope.output_coverage.per_track;
coverage_track = reshape([coverage.track_id], 1, []);
[has_coverage, coverage_index] = ismember(ids, coverage_track);
for q = 1:n_id
    d = track_scope_detail_template(); d.applicable = applicable(q);
    d.min_labeled_assoc = scope.track_accuracy.min_labeled_assoc;
    d.purity_threshold = scope.track_accuracy.purity_threshold;
    if applicable(q)
        c = scope_index(q);
        d.n_labeled_assoc = scope.accuracy.output_labeled_counts(c);
        d.is_correct = scope.track_accuracy.is_correct(c);
    end
    if has_pair(q)
        pair = pairs(pair_index(q)); d.has_match = true;
        d.matched_truth_id = pair.truth_id; d.n_correct = pair.n_correct;
        d.n_inconsistent = max(d.n_labeled_assoc - d.n_correct, 0);
        d.association_consistency = pair.association_consistency;
        d.truth_total = pair.truth_total;
        d.coverage = pair.coverage;
        d.n_truth_not_correct = max(d.truth_total - d.n_correct, 0);
    end
    if has_coverage(q)
        oc = coverage(coverage_index(q)); d.formal_output_coverage = oc.rate;
        d.n_formal_output_covered = oc.n_covered_measurements;
        d.n_formal_output_reference = oc.n_reference_measurements;
    end
    d.track_level_score = double(d.is_correct); detail(q) = d;
end
end

function count = grouped_count(group, mask, n_group)
group = group(mask);
if isempty(group), count = zeros(n_group, 1); return; end
count = accumarray(group(:), 1, [n_group, 1]);
end

function [angle, position] = grouped_error_metrics( ...
        ids, angle_track, az_error, el_error, position_track, position_error, los_error)
n = numel(ids);
angle = repmat(angle_metrics([], [], []), n, 1);
position = repmat(position_metrics(zeros(3, 0)), n, 1);
[found, group] = ismember(angle_track, ids);
if any(found)
    group = group(found); az = az_error(found); el = el_error(found);
    count = accumarray(group(:), 1, [n, 1]);
    az_square = accumarray(group(:), az(:).^2, [n, 1]);
    el_square = accumarray(group(:), el(:).^2, [n, 1]);
    los = los_error(found);
    los_square = accumarray(group(:), los(:).^2, [n, 1]);
    for q = find(count > 0).'
        angle(q).n = count(q); angle(q).rmse_az_deg = sqrt(az_square(q) / count(q));
        angle(q).rmse_el_deg = sqrt(el_square(q) / count(q));
        angle(q).rmse_los_deg = sqrt(los_square(q) / count(q));
    end
end
[found, group] = ismember(position_track, ids);
if any(found)
    group = group(found); error = position_error(:, found);
    count = accumarray(group(:), 1, [n, 1]); sums = zeros(3, n);
    for axis = 1:3
        values = reshape(error(axis, :).^2, [], 1);
        sums(axis, :) = accumarray(group(:), values, [n, 1]).';
    end
    for q = find(count > 0).'
        position(q).n = count(q);
        position(q).rmse_e_m = sqrt(sums(1, q) / count(q));
        position(q).rmse_n_m = sqrt(sums(2, q) / count(q));
        position(q).rmse_u_m = sqrt(sums(3, q) / count(q));
        position(q).rmse_3d_m = sqrt(sum(sums(:, q)) / count(q));
    end
end
end

function d = track_detail_template()
d = struct('track_id', NaN, 'n_output_points', 0, 'n_output_events', 0, ...
    'n_output_2d', 0, 'n_output_3d', 0, 'start_time_s', NaN, ...
    'end_time_s', NaN, 'duration_s', NaN, 'n_associations', 0, ...
    'n_active_range_assoc', 0, 'n_active_angle_assoc', 0, ...
    'n_passive_assoc', 0, 'two_d', track_scope_detail_template(), ...
    'three_d', track_scope_detail_template(), ...
    'overall', track_scope_detail_template(), ...
    'primary_scope', '', 'primary', track_scope_detail_template(), ...
    'angle_rmse', struct('n', 0, 'az_deg', NaN, 'el_deg', NaN, 'los_deg', NaN), ...
    'position_rmse', struct('n', 0, 'e_m', NaN, 'n_m', NaN, ...
    'u_m', NaN, 'three_d_m', NaN));
end

function d = track_scope_detail_template()
d = struct('applicable', false, 'has_match', false, ...
    'matched_truth_id', NaN, 'n_labeled_assoc', 0, 'n_correct', 0, ...
    'n_inconsistent', 0, 'association_consistency', NaN, ...
    'truth_total', 0, 'coverage', NaN, 'n_truth_not_correct', 0, ...
    'formal_output_coverage', NaN, 'n_formal_output_covered', 0, ...
    'n_formal_output_reference', 0, 'purity_threshold', NaN, ...
    'min_labeled_assoc', 0, 'is_correct', false, 'track_level_score', 0);
end

function audit = attach_evaluation_identity_audit(est, metrics)
audit = struct('status', 'unavailable', ...
    'reason', 'filter_output_identity_audit_missing');
if ~isstruct(est) || ~isfield(est, 'output_identity_audit') || ...
        isempty(est.output_identity_audit)
    return;
end
audit = est.output_identity_audit;
ids3 = reshape(metrics.three_d.output.track_ids, 1, []);
ids2 = reshape(metrics.two_d.output.track_ids, 1, []);
if ~isequal(sort(reshape(audit.public_3d_ids, 1, [])), sort(ids3)) || ...
        ~isequal(sort(reshape(audit.public_2d_ids, 1, [])), sort(ids2))
    error('evaluate_joint_tracking_metrics:IdentityAuditOutputMismatch', ...
        '滤波输出身份账本与评价器二维/三维公开ID集合不一致。');
end
rows = audit.three_d_rows;
for q = 1:numel(rows)
    rows(q).matched_truth_id = NaN;
    rows(q).truth_coverage = NaN;
    rows(q).is_correct = false;
    i = find([metrics.track_details.track_id] == rows(q).public_id, 1);
    if isempty(i), continue; end
    detail = metrics.track_details(i).three_d;
    rows(q).matched_truth_id = detail.matched_truth_id;
    rows(q).truth_coverage = detail.coverage;
    rows(q).is_correct = detail.is_correct;
end
audit.three_d_rows = rows;
audit.status = 'ok';
audit.reason = '';
audit.evaluation_3d_public_ids = ids3;
audit.evaluation_2d_public_ids = ids2;
audit.evaluation_consistent = true;
end

function validate_output_id_domains(data, cfg)
offset = round(get_cfg(cfg, 'joint_2d_id_offset', 1000));
ids2 = unique(data.output_track(data.output_dim == 2 & isfinite(data.output_track)));
ids3 = unique(data.output_track(data.output_dim == 3 & isfinite(data.output_track)));
if any(ids3 < 1 | ids3 > offset) || any(ids2 <= offset)
    error('evaluate_joint_tracking_metrics:OutputIdDomainViolation', ...
        '评价输入违反输出ID分域：三维必须位于1..%d，二维必须从%d开始。', ...
        offset, offset + 1);
end
if ~isempty(intersect(ids2, ids3))
    error('evaluate_joint_tracking_metrics:MixedOutputIdDomain', ...
        '同一个公开输出ID同时属于二维和三维。');
end
end

function result = assert_dimension_evaluation_contract(metrics)
assert_measurement_metric_contract(metrics.two_d, '二维');
assert_measurement_metric_contract(metrics.three_d, '三维');
assert_measurement_metric_contract(metrics.overall, '总体');
expected_2d_input = metrics.two_d.reference.n_passive_input + ...
    metrics.two_d.reference.n_active_ae_only_input;
if metrics.two_d.association.n_measurements ~= expected_2d_input
    error('evaluate_joint_tracking_metrics:TwoDimensionalInputMismatch', ...
        '二维评价分母不等于物理被动AE与主动AE-only输入之和。');
end
details = metrics.track_details;
ids = reshape([details.track_id], 1, []);
ids2 = reshape(metrics.two_d.track_accuracy.track_ids, 1, []);
ids3 = reshape(metrics.three_d.track_accuracy.track_ids, 1, []);
if ~isempty(intersect(ids2, ids3))
    error('evaluate_joint_tracking_metrics:ScopeIdOverlap', ...
        '二维与三维评价作用域包含重复公开航迹ID。');
end
if ~all(ismember([ids2, ids3], ids)) || numel(ids) ~= numel(unique([ids2, ids3]))
    error('evaluate_joint_tracking_metrics:TrackDetailCoverage', ...
        '单轨评价明细与二维/三维总体输出ID集合不一致。');
end
correct2 = 0; correct3 = 0;
for q = 1:numel(details)
    if details(q).n_output_2d > 0
        if ~strcmp(details(q).primary_scope, 'two_d') || details(q).n_output_3d > 0
            error('evaluate_joint_tracking_metrics:InvalidTwoDimensionalPrimaryScope', ...
                '二维公开ID的单轨主评价范围不是two_d。');
        end
        correct2 = correct2 + double(details(q).two_d.is_correct);
    elseif details(q).n_output_3d > 0
        if ~strcmp(details(q).primary_scope, 'three_d')
            error('evaluate_joint_tracking_metrics:InvalidThreeDimensionalPrimaryScope', ...
                '三维公开ID的单轨主评价范围不是three_d。');
        end
        correct3 = correct3 + double(details(q).three_d.is_correct);
    end
end
if correct2 ~= metrics.two_d.track_accuracy.n_correct_tracks || ...
        correct3 ~= metrics.three_d.track_accuracy.n_correct_tracks
    error('evaluate_joint_tracking_metrics:SingleTrackAggregateMismatch', ...
        '单轨正确判定求和与二维/三维总体正确航迹数不一致。');
end
if metrics.overall.track_accuracy.n_correct_tracks ~= correct2 + correct3 || ...
        metrics.overall.output.n_unique_tracks ~= numel(ids2) + numel(ids3)
    error('evaluate_joint_tracking_metrics:OverallAggregateMismatch', ...
        '总体分域汇总不是二维与三维评价结果的严格加和。');
end
result = struct('status', 'ok', 'id_scopes_disjoint', true, ...
    'single_track_matches_scope_totals', true, ...
    'overall_is_dimension_sum', true, ...
    'measurement_formulas_verified', true, ...
    'two_d_input_includes_active_ae_only', true, ...
    'identity_audit_matches_evaluation', ...
        isfield(metrics.output_identity_audit, 'evaluation_consistent') && ...
        metrics.output_identity_audit.evaluation_consistent, ...
    'n_2d_correct', correct2, 'n_3d_correct', correct3);
end

function assert_measurement_metric_contract(scope, label)
n_input = scope.association.n_measurements;
n_assigned = scope.association.n_assigned;
n_correct = scope.accuracy.n_correct;
if scope.measurement_accuracy.n_measurements ~= n_input || ...
        scope.measurement_accuracy.n_correct ~= n_correct || ...
        scope.measurement_accuracy.n_not_correct ~= n_input - n_correct || ...
        ~isequaln(scope.measurement_accuracy.rate, safe_ratio(n_correct, n_input))
    error('evaluate_joint_tracking_metrics:MeasurementAccuracyMismatch', ...
        '%s量测正确率不满足正确关联数/输入量测数。', label);
end
if scope.association_consistency.n_assigned ~= n_assigned || ...
        scope.association_consistency.n_correct ~= n_correct || ...
        scope.association_consistency.n_not_correct ~= n_assigned - n_correct || ...
        ~isequaln(scope.association_consistency.rate, ...
        safe_ratio(n_correct, n_assigned))
    error('evaluate_joint_tracking_metrics:AssociationConsistencyMismatch', ...
        '%s已关联一致率不满足正确关联数/已关联量测数。', label);
end
end

function dim = snapshot_dimension(tr)
dim = 0;
if isfield(tr, 'mode') && strncmp(tr.mode, '2d', 2)
    dim = 2;
elseif isfield(tr, 'mode') && strncmp(tr.mode, '3d', 2)
    dim = 3;
elseif isfield(tr, 'state3d') && all(isfinite(tr.state3d))
    dim = 3;
elseif isfield(tr, 'angle_state') && all(isfinite(tr.angle_state))
    dim = 2;
end
end

function d = angle_diff(a, b)
d = mod(a - b + 180, 360) - 180;
end

function x = sized_row(x0, n, fill)
x = fill * ones(1, n);
if n == 0 || isempty(x0), return; end
m = min(n, numel(x0));
x(1:m) = reshape(x0(1:m), 1, []);
end

function has_range = measurement_has_range(meas, n)
has_range = false(1, n);
if n == 0 || ~isstruct(meas) || ~isfield(meas, 'has_range') || ...
        isempty(meas.has_range)
    return;
end
m = min(n, numel(meas.has_range));
has_range(1:m) = logical(reshape(meas.has_range(1:m), 1, []));
end

function t = measurement_times(t0, n, fallback)
t = fallback * ones(1, n);
if n == 0 || isempty(t0), return; end
if isscalar(t0)
    t(:) = t0;
else
    m = min(n, numel(t0));
    t(1:m) = reshape(t0(1:m), 1, []);
end
end

function value = indexed_value(x, i, fallback)
if i >= 1 && i <= numel(x), value = x(i); else, value = fallback; end
end

function value = indexed_field_value(s, name, i, fallback)
value = fallback;
if isstruct(s) && isfield(s, name)
    value = indexed_value(s.(name), i, fallback);
end
end

function value = indexed_text(s, name, i, fallback)
value = fallback;
if ~isstruct(s) || ~isfield(s, name), return; end
x = s.(name);
if iscell(x) && i >= 1 && i <= numel(x) && ischar(x{i})
    value = x{i};
elseif ischar(x) && i == 1
    value = x;
end
end

function value = stat_value(stats, name, fallback)
if isstruct(stats) && isfield(stats, name) && isscalar(stats.(name)) && ...
        isfinite(stats.(name))
    value = stats.(name);
else
    value = fallback;
end
end

function value = stat_sum_or(stats, names, fallback)
value = 0;
for q = 1:numel(names)
    name = names{q};
    if ~isstruct(stats) || ~isfield(stats, name) || ...
            ~isscalar(stats.(name)) || ~isfinite(stats.(name))
        value = fallback;
        return;
    end
    value = value + stats.(name);
end
end

function ids = collect_confirmed_ids(est, formal_output_ids)
% Every formal output has already passed its branch confirmation rule.
% Start from those IDs, then add confirmed-but-not-output states (for
% example a confirmed logical track entering hold) from transitions and
% diagnostic snapshots.  These sources must be combined: the mature 3-D
% backbone does not emit logical_confirm_* transitions for tracks that are
% confirmed directly by the 3-D manager.
ids = reshape(formal_output_ids, 1, []);

% Current joint-filter results retain transition_log independently of
% history_level.  The logical_confirm_* transitions supplement formal
% outputs with logical IDs that reached confirmation directly in hold.
if isstruct(est) && isfield(est, 'transition_log') && ~isempty(est.transition_log)
    log = est.transition_log;
    if isfield(log, 'id') && isfield(log, 'reason')
        reasons = {log.reason};
        is_confirm = false(size(reasons));
        for q = 1:numel(reasons)
            reason = reasons{q};
            is_confirm(q) = ischar(reason) && strncmp(reason, 'logical_confirm_', 16);
        end
        if any(is_confirm)
            ids = [ids, reshape([log(is_confirm).id], 1, [])]; %#ok<AGROW>
        end
    end
end

% Diagnostic/full snapshots also retain confirmed states which can be
% temporarily withheld from formal output by freshness or hold rules.
if isstruct(est) && isfield(est, 'logical_tracks')
    counts = zeros(numel(est.logical_tracks), 1);
    for k = 1:numel(est.logical_tracks)
        tracks = est.logical_tracks{k};
        if isempty(tracks) || ~isfield(tracks, 'id') || ~isfield(tracks, 'confirmed')
            continue;
        end
        counts(k) = nnz(logical([tracks.confirmed]));
    end
    snapshot_ids = nan(1, sum(counts)); p = 0;
    for k = 1:numel(est.logical_tracks)
        if counts(k) == 0, continue; end
        tracks = est.logical_tracks{k}; confirmed = logical([tracks.confirmed]);
        values = reshape([tracks(confirmed).id], 1, []);
        ii = p + (1:numel(values)); snapshot_ids(ii) = values; p = p + numel(values);
    end
    ids = [ids, snapshot_ids]; %#ok<AGROW>
end
ids = unique(ids(isfinite(ids)));
end

function n = get_transition_count(est)
if isfield(est, 'transition_log'), n = numel(est.transition_log); else, n = 0; end
end

function v = safe_ratio(a, b)
if b > 0, v = a / b; else, v = NaN; end
end

function v = mean_or_nan(x)
if isempty(x), v = NaN; else, v = mean(x); end
end

function v = get_cfg(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name)), v = cfg.(name); else, v = fallback; end
end

function data = empty_metric_data()
data = struct('n_meas_2d', 0, 'n_meas_3d', 0, ...
    'n_active_measurements', 0, 'n_passive_measurements', 0, ...
    'n_active_range_measurements', 0, 'n_active_ae_only_measurements', 0, ...
    'truth_2d_id', zeros(1, 0), 'truth_2d_t', zeros(1, 0), ...
    'truth_2d_event', zeros(1, 0), 'truth_2d_az', zeros(1, 0), ...
    'truth_2d_el', zeros(1, 0), 'truth_2d_input_dim', zeros(1, 0), ...
    'truth_2d_is_passive', false(1, 0), 'truth_2d_source', zeros(1, 0), ...
    'truth_3d_id', zeros(1, 0), 'truth_3d_t', zeros(1, 0), ...
    'truth_3d_event', zeros(1, 0), 'truth_3d_az', zeros(1, 0), ...
    'truth_3d_el', zeros(1, 0), 'truth_3d_xyz', zeros(3, 0), ...
    'assoc_track', zeros(1, 0), 'assoc_truth', zeros(1, 0), ...
    'assoc_meas_dim', zeros(1, 0), 'assoc_filter_dim', zeros(1, 0), ...
    'assoc_input_dim', zeros(1, 0), 'assoc_source', zeros(1, 0), ...
    'assoc_event', zeros(1, 0), 'assoc_t', zeros(1, 0), ...
    'assoc_is_active', false(1, 0), ...
    'assoc_is_passive', false(1, 0), 'assoc_range_updated', false(1, 0), ...
    'output_track', zeros(1, 0), 'output_truth', zeros(1, 0), ...
    'output_dim', zeros(1, 0), 'output_t', zeros(1, 0), ...
    'output_event', zeros(1, 0), 'output_az', zeros(1, 0), ...
    'output_el', zeros(1, 0), 'output_pos', zeros(3, 0));
end

function data = preallocate_metric_data(n2, n3, na, no, n2_event, n3_event)
data = empty_metric_data();
data.truth_2d_id = nan(1, n2); data.truth_2d_t = nan(1, n2);
data.truth_2d_event = zeros(1, n2); data.truth_2d_az = nan(1, n2);
data.truth_2d_el = nan(1, n2);
data.truth_2d_input_dim = zeros(1, n2); data.truth_2d_is_passive = false(1, n2);
data.truth_2d_source = zeros(1, n2);
data.truth_3d_id = nan(1, n3); data.truth_3d_t = nan(1, n3);
data.truth_3d_event = zeros(1, n3); data.truth_3d_az = nan(1, n3);
data.truth_3d_el = nan(1, n3); data.truth_3d_xyz = nan(3, n3);
data.truth_2d_offsets = [1; 1 + cumsum(n2_event(:))];
data.truth_3d_offsets = [1; 1 + cumsum(n3_event(:))];
data.assoc_track = nan(1, na); data.assoc_truth = nan(1, na);
data.assoc_meas_dim = zeros(1, na); data.assoc_filter_dim = zeros(1, na);
data.assoc_input_dim = zeros(1, na);
data.assoc_source = zeros(1, na);
data.assoc_event = zeros(1, na); data.assoc_t = nan(1, na);
data.assoc_is_active = false(1, na);
data.assoc_is_passive = false(1, na); data.assoc_range_updated = false(1, na);
data.output_track = nan(1, no); data.output_truth = nan(1, no);
data.output_dim = zeros(1, no); data.output_t = nan(1, no);
data.output_event = zeros(1, no); data.output_az = nan(1, no);
data.output_el = nan(1, no); data.output_pos = nan(3, no);
end

function e = empty_event()
e = struct('t_sec', NaN, ...
    'active', struct('n_meas', 0, 't_sec', zeros(1, 0), ...
        'ids', zeros(1, 0), 'has_range', false(1, 0), ...
        'rae', zeros(3, 0), 'xyz', zeros(3, 0)), ...
    'passive', struct('n_meas', 0, 't_sec', zeros(1, 0), ...
        'ids', zeros(1, 0), 'ang', zeros(2, 0), 'kind', zeros(1, 0)));
end
