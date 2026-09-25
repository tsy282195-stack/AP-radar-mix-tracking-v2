function flag = parse_validity_flag(raw_value)
%PARSE_VALIDITY_FLAG Parse numeric or Chinese measurement-validity tokens.
% Returns 1 for valid, 0 for invalid, and NaN for an unrecognized token.
% Supported inputs include numeric 1/0 and text 有效/无效. Numeric values
% retain the historical rule that values greater than or equal to 0.5 are
% valid. Text matching is exact so that 无效 can never be mistaken for 有效.

flag = NaN;

if islogical(raw_value)
    if isscalar(raw_value)
        flag = double(raw_value);
    end
    return;
end

if isnumeric(raw_value)
    if isscalar(raw_value) && isfinite(raw_value)
        flag = double(raw_value >= 0.5);
    end
    return;
end

if isa(raw_value, 'string')
    if ~isscalar(raw_value)
        return;
    end
    raw_value = char(raw_value);
end
if ~ischar(raw_value)
    return;
end

token = strtrim(strrep(raw_value, char(65279), ''));
if numel(token) >= 2 && ...
        ((token(1) == char(34) && token(end) == char(34)) || ...
         (token(1) == char(39) && token(end) == char(39)))
    token = strtrim(token(2:end - 1));
end
if isempty(token)
    return;
end

numeric_value = str2double(token);
if isfinite(numeric_value)
    flag = double(numeric_value >= 0.5);
    return;
end

switch token
    case '有效'
        flag = 1;
    case '无效'
        flag = 0;
end
end
