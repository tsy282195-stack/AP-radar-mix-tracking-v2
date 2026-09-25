function az = azimuth_for_plot(az, center_deg)
%AZIMUTH_FOR_PLOT Re-center azimuth for display without changing filtering.
% center_deg=0 gives [-180,180); center_deg=180 gives [0,360).

if nargin < 2 || isempty(center_deg), center_deg = 0; end
if ~isscalar(center_deg) || ~isfinite(center_deg) || ...
        ~any(abs(center_deg - [0, 180]) <= 1e-9)
    error('azimuth_for_plot:InvalidCenter', 'center_deg must be 0 or 180.');
end
finite = isfinite(az);
az(finite) = mod(az(finite) - center_deg + 180, 360) - 180 + center_deg;
end
