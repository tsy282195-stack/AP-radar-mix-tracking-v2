function info = save_all_open_figures(save_dir, figures, opts)
%SAVE_ALL_OPEN_FIGURES Save selected figures in configured formats.
% figures omitted/empty keeps legacy behavior and selects all open figures.

if nargin < 1 || isempty(save_dir)
    info = empty_save_info(save_dir);
    return;
end
if nargin < 2 || isempty(figures)
    figures = findall(0, 'Type', 'figure');
end
if nargin < 3 || isempty(opts), opts = struct(); end
formats = get_opt(opts, 'plot_save_formats', {'png'});
if ischar(formats) || (isa(formats, 'string') && isscalar(formats))
    formats = {char(formats)};
elseif isa(formats, 'string')
    formats = cellstr(formats(:).');
end
formats = cellfun(@(x) lower(char(x)), formats, 'UniformOutput', false);
formats = unique(formats, 'stable');
dpi = max(1, round(get_opt(opts, 'plot_png_resolution', 150)));
close_after_save = logical(get_opt(opts, 'plot_close_after_save', false));

if exist(save_dir, 'dir') ~= 7, mkdir(save_dir); end
figures = reshape(figures, 1, []);
valid = arrayfun(@(h) isgraphics(h, 'figure'), figures);
figures = figures(valid);
info = empty_save_info(save_dir);
info.n_figures = numel(figures);
info.formats = formats;
if isempty(figures), return; end

save_tic = tic;
for i = 1:numel(figures)
    fig = figures(i);
    if ~isgraphics(fig, 'figure'), continue; end
    try
        name = get(fig, 'Name');
    catch
        continue;
    end
    if isempty(name), name = sprintf('figure_%02d', i); end
    safe_name = regexprep(name, '[\\/:*?"<>|]', '_');
    base_path = fullfile(save_dir, sprintf('%02d_%s', i, safe_name));
    figure_ok = true;
    for q = 1:numel(formats)
        format = formats{q};
        try
            if strcmp(format, 'fig')
                savefig(fig, [base_path, '.fig']);
            elseif strcmp(format, 'png')
                png_path = [base_path, '.png'];
                try
                    exportgraphics(fig, png_path, 'Resolution', dpi);
                catch
                    saveas(fig, png_path);
                end
            end
            info.n_files = info.n_files + 1;
        catch ME
            figure_ok = false;
            warning('保存%s失败: %s.%s (%s)', upper(format), ...
                base_path, format, ME.message);
        end
    end
    if figure_ok, info.n_figures_succeeded = info.n_figures_succeeded + 1; end
    if close_after_save && isgraphics(fig, 'figure'), close(fig); end
end
info.elapsed_s = toc(save_tic);
fprintf('图像已保存到: %s（%d/%d个图窗，格式=%s，耗时%.2fs）\n', ...
    save_dir, info.n_figures_succeeded, info.n_figures, ...
    strjoin(formats, '+'), info.elapsed_s);
end

function value = get_opt(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end

function info = empty_save_info(save_dir)
info = struct('save_dir', save_dir, 'formats', {cell(1, 0)}, ...
    'n_figures', 0, 'n_figures_succeeded', 0, 'n_files', 0, ...
    'elapsed_s', 0);
end
