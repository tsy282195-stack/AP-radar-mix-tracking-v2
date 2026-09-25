function d = joint_measurement_disposition( ...
        n, a, type, suppressed, event_index, suppression_reason)
%JOINT_MEASUREMENT_DISPOSITION Reconcile every filter input exactly once.
% Array position is the measurement index in events(k).active/passive.
% action: 1=associated, 2=birth, 3=explicitly suppressed.  The reason
% field records the explicit suppression policy. Confirmation/formal output
% is not required.
% input_dim is the branch before lifecycle/capacity pruning; filter_dim is
% the retained branch. Suppressed inputs did not enter either branch (0).
assert(numel(suppressed) == n && islogical(suppressed), ...
    'run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
    'Suppression mask must identify the input measurements.');
if nargin < 6 || isempty(suppression_reason)
    suppression_reason = repmat({'suppressed_unassociated'}, 1, n);
elseif ischar(suppression_reason) || ...
        (isa(suppression_reason, 'string') && isscalar(suppression_reason))
    suppression_reason = repmat({char(suppression_reason)}, 1, n);
elseif iscell(suppression_reason)
    suppression_reason = reshape(suppression_reason, 1, []);
else
    suppression_reason = reshape(cellstr(suppression_reason), 1, []);
end
if numel(suppression_reason) ~= n
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Suppression reasons must match the input measurement count.');
end
d = struct('action', zeros(1, n, 'uint8'), 'track_id', zeros(1, n), ...
    'filter_dim', zeros(1, n, 'uint8'), 'input_dim', zeros(1, n, 'uint8'), ...
    'range_updated', false(1, n), 'reason', {repmat({''}, 1, n)});
d.action(suppressed) = 3;
d.reason(suppressed) = suppression_reason(suppressed);
q = find(cellfun(@(value) association_type_matches(value, type), a.type));
mi = a.meas_index(q);
invalid = ~isfinite(mi) | mi < 1 | mi > n | mi ~= round(mi);
if any(invalid)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s has invalid measurement indices: %s (n=%d).', ...
        event_index, type, mat2str(mi(invalid)), n);
end

[unique_mi, ~, group] = unique(mi, 'stable');
counts = accumarray(group(:), 1);
duplicate_mi = unique_mi(counts > 1);
if ~isempty(duplicate_mi)
    duplicate_entries = ismember(mi, duplicate_mi);
    duplicate_types = strjoin(a.type(q(duplicate_entries)), ',');
    duplicate_ids = a.id(q(duplicate_entries));
    duplicate_dims = a.filter_dim(q(duplicate_entries));
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        ['Event %d %s has duplicate measurement indices %s ' ...
        '(types: %s; track IDs: %s; filter dims: %s).'], ...
        event_index, type, mat2str(duplicate_mi), duplicate_types, ...
        mat2str(duplicate_ids), mat2str(duplicate_dims));
end
if any(d.action(mi) ~= 0)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s is both associated and suppressed.', event_index, type);
end
d.action(mi) = uint8(1 + strcmp(a.type(q), [type '_birth']));
d.reason(mi) = repmat({'associated'}, 1, numel(mi));
is_birth = strcmp(a.type(q), [type '_birth']);
d.reason(mi(is_birth)) = repmat({'track_birth'}, 1, nnz(is_birth));
d.track_id(mi) = a.id(q);
d.filter_dim(mi) = uint8(a.filter_dim(q));
d.input_dim(mi) = d.filter_dim(mi);
if isfield(a, 'input_dim'), d.input_dim(mi) = uint8(a.input_dim(q)); end
d.range_updated(mi) = a.range_updated(q);
if any(d.action == 0)
    error('run_filter_joint_2d3d:MeasurementDispositionMismatch', ...
        'Event %d %s contains measurements without a disposition.', event_index, type);
end
end

function tf = association_type_matches(value, requested)
if isa(value, 'string') && isscalar(value), value = char(value); end
tf = ischar(value) && ~isempty(strfind(value, requested)); %#ok<STREMP>
end
