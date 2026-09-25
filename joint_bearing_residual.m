function residual = joint_bearing_residual(measured_ae_deg, predicted_ae_deg)
%JOINT_BEARING_RESIDUAL Coupled azimuth/elevation residual in degrees.
% The two rows always describe one physical bearing point.  Azimuth is
% wrapped to [-180, 180); elevation remains a signed linear difference.

if size(measured_ae_deg, 1) ~= 2 || size(predicted_ae_deg, 1) ~= 2
    error('joint_bearing_residual:InvalidShape', ...
        'Measured and predicted bearings must be 2-by-N [azimuth;elevation].');
end
n_measured = size(measured_ae_deg, 2);
n_predicted = size(predicted_ae_deg, 2);
if n_predicted == 1 && n_measured ~= 1
    predicted_ae_deg = repmat(predicted_ae_deg, 1, n_measured);
elseif n_measured ~= n_predicted
    error('joint_bearing_residual:ColumnMismatch', ...
        'Predicted bearing must have one column or match the measured columns.');
end
residual = measured_ae_deg - predicted_ae_deg;
residual(1, :) = mod(residual(1, :) + 180, 360) - 180;
end
