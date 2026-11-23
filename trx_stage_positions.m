function S = trx_stage_positions(recording, stageLog, pngFile)
%TRX_STAGE_POSITIONS  Stage position of every TRx sweep of a radar recording.
%   S = TRX_STAGE_POSITIONS(RECORDING, STAGELOG) loads a recording of
%   radar_gui.py (.mat) and the stage position log of actuator_logger.m
%   (.csv), takes the sweeps of trace 0 (TRx), interpolates the stage
%   position at the start of every sweep and plots
%     - the logged stage positions and the position of every sweep,
%     - the stage speed at every sweep,
%     - the range profiles of the sweeps over the stage position.
%   S = TRX_STAGE_POSITIONS() takes the newest recording in Results and the
%   stage log in Results that covers it; TRX_STAGE_POSITIONS(..., PNGFILE)
%   also saves the figure.
%
%   S.t_s          [1 x N_az] start of every sweep, POSIX s UTC
%   S.position_mm  [1 x N_az] stage position at every sweep (linear
%                  interpolation of the stage log; NaN outside it)
%   S.signal       [N_samples x N_az] raw IF samples of trace 0 (int16)
%   S.range_m      [1 x N_samples] one-way range of the FFT bins
%   S.segment      stage coordinate segment of the recording
%   S.recording, S.stage_log   the files used
%
%   See also actuator_logger.

    results = fullfile(fileparts(mfilename('fullpath')), 'Results');
    if nargin < 1 || isempty(recording)
        recording = newestFile(fullfile(results, 'acquisition_*.mat'));
    end
    R = load(recording, 'radar_signal', 'radar_rx', 'radar_timestamps', ...
             'radar_range_samples', 'radar_full_scale');
    trx = find(R.radar_rx(:)' == 0, 1);
    if isempty(trx)
        error('trx_stage_positions:trace', '%s holds no trace 0 (TRx).', recording);
    end
    signal = reshape(R.radar_signal(:, trx, :), size(R.radar_signal, 1), []);
    t = double(R.radar_timestamps(:)');

    if nargin < 2 || isempty(stageLog)
        stageLog = coveringLog(results, t);
    end
    A = readStageLog(stageLog);
    pos = ~isnan(A.timestamp_s) & ~isnan(A.position_mm);
    near = pos & A.timestamp_s >= t(1) - 10 & A.timestamp_s <= t(end) + 10;
    if ~any(near)
        error('trx_stage_positions:log', '%s has no stage position during %s.', ...
              stageLog, recording);
    end
    segments = unique(A.segment(near))';
    segment = A.segment(find(near, 1));
    if numel(segments) > 1
        warning('trx_stage_positions:segment', ['The stage coordinates changed during the ' ...
                'recording (segments %s): only segment %d is used.'], mat2str(segments), segment);
    end
    rows = pos & A.segment == segment;
    x = interp1(A.timestamp_s(rows), A.position_mm(rows), t);

    S = struct('t_s', t, 'position_mm', x, 'signal', signal, ...
               'range_m', double(R.radar_range_samples(:)'), 'segment', segment, ...
               'recording', recording, 'stage_log', stageLog);

    describe(S, A, rows);
    fig = plotPositions(S, A, rows, double(R.radar_full_scale));
    if nargin >= 3 && ~isempty(pngFile)
        exportgraphics(fig, pngFile, 'Resolution', 150);
    end
end

% -------------------------------------------------------------------------
function file = newestFile(pattern)
    f = dir(pattern);
    if isempty(f)
        error('trx_stage_positions:file', 'No file %s.', pattern);
    end
    [~, k] = max([f.datenum]);
    file = fullfile(f(k).folder, f(k).name);
end

function A = readStageLog(file)
    opts = detectImportOptions(file, 'Delimiter', ',', 'FileType', 'text');
    opts = setvartype(opts, {'timestamp_s', 'position_mm', 'segment'}, 'double');
    opts = setvartype(opts, {'source', 'utc', 'note'}, 'string');
    A = readtable(file, opts);
end

function file = coveringLog(folder, t)
    % The stage log with the most positions during t(1) ... t(end).
    f = dir(fullfile(folder, 'actuator_*.csv'));
    best = 0;
    file = '';
    for k = 1:numel(f)
        name = fullfile(f(k).folder, f(k).name);
        A = readStageLog(name);
        n = nnz(~isnan(A.position_mm) & A.timestamp_s >= t(1) & A.timestamp_s <= t(end));
        if n > best
            best = n;
            file = name;
        end
    end
    if isempty(file)
        error('trx_stage_positions:log', 'No stage log in %s covers the recording.', folder);
    end
end

function describe(S, A, rows)
    t = S.t_s;
    x = S.position_mm;
    v = diff(x)./diff(t);
    moving = abs(v) > 0.05;
    fprintf('Recording  %s: %d TRx sweeps, %.1f s\n', S.recording, numel(t), t(end) - t(1));
    fprintf('Stage log  %s: segment %d, %d positions during the recording\n', S.stage_log, ...
            S.segment, nnz(rows & A.timestamp_s >= t(1) & A.timestamp_s <= t(end)));
    fprintf('Positions  %.3f to %.3f mm; %d sweeps without a position (outside the log)\n', ...
            min(x), max(x), nnz(isnan(x)));
    if any(moving)
        fprintf('Speed      median %.3f mm/s while moving (%d sweeps)\n', median(abs(v(moving))), ...
                nnz(moving));
    end
    w = startsWith(A.note, "Warning") & A.timestamp_s >= t(1) & A.timestamp_s <= t(end);
    for k = find(w)'
        fprintf('%s\n', A.note(k));
    end
end

function fig = plotPositions(S, A, rows, fullScale)
    t0 = S.t_s(1);
    ts = S.t_s - t0;
    x = S.position_mm;
    fig = figure('Name', 'TRx sweeps and stage positions', 'Position', [100 100 1100 900]);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');
    [~, recName] = fileparts(S.recording);
    [~, logName] = fileparts(S.stage_log);
    title(tl, sprintf('%s  +  %s (TRx, trace 0)', recName, logName), 'Interpreter', 'none');

    ax1 = nexttile(tl);
    hold(ax1, 'on');
    plot(ax1, ts, x, '-', 'LineWidth', 1.5, 'DisplayName', 'position of every sweep (interpolated)');
    sources = ["move", "idle", "halt"];
    markers = {'o', 's', 'x'};
    for k = 1:numel(sources)
        r = rows & A.source == sources(k);
        if any(r)
            plot(ax1, A.timestamp_s(r) - t0, A.position_mm(r), markers{k}, 'MarkerSize', 4, ...
                 'DisplayName', sprintf('logged: %s', sources(k)));
        end
    end
    w = startsWith(A.note, "Warning");
    for k = find(w)'
        xline(ax1, A.timestamp_s(k) - t0, 'r--', 'HandleVisibility', 'off');
    end
    xlim(ax1, [ts(1) ts(end)]);
    grid(ax1, 'on');
    xlabel(ax1, 'Time since the first sweep (s)');
    ylabel(ax1, 'Stage position (mm)');
    legend(ax1, 'Location', 'best');

    ax2 = nexttile(tl);
    v = [NaN diff(x)./diff(S.t_s)];
    plot(ax2, ts, v, '-');
    xlim(ax2, [ts(1) ts(end)]);
    grid(ax2, 'on');
    xlabel(ax2, 'Time since the first sweep (s)');
    ylabel(ax2, 'Stage speed (mm/s)');

    ax3 = nexttile(tl);
    [image, xg, range] = rangeImage(S, fullScale);
    if isempty(image)
        text(ax3, 0.5, 0.5, 'The stage did not move during the recording.', 'Units', 'normalized', ...
             'HorizontalAlignment', 'center');
        axis(ax3, 'off');
        return
    end
    imagesc(ax3, xg, range, image);
    axis(ax3, 'xy');
    peak = max(image(:));
    clim(ax3, [peak - 60, peak]);
    cb = colorbar(ax3);
    cb.Label.String = 'dBFS';
    xlabel(ax3, 'Stage position (mm)');
    ylabel(ax3, 'One-way range (m)');
    title(ax3, 'TRx range profiles of the sweeps while the stage moves');
end

function [image, xg, range] = rangeImage(S, fullScale)
    % Range profiles (dBFS, as radar_gui.py shows them) of the sweeps of the
    % longest move, on an even grid of stage positions.
    x = S.position_mm;
    moving = [false, abs(diff(x)) > 0];
    edges = diff([0, moving, 0]);
    first = find(edges == 1);
    last = find(edges == -1) - 1;
    image = [];
    xg = [];
    range = [];
    if isempty(first)
        return
    end
    [~, k] = max(last - first);
    sweeps = first(k) - 1:last(k);
    xs = x(sweeps);

    n = size(S.signal, 1);
    nb = find(S.range_m <= min(4, S.range_m(floor(n/2) + 1)), 1, 'last');
    range = S.range_m(1:nb);
    alpha = (n - 1)/2;
    w = besseli(0, 2.5*sqrt(1 - (((0:n-1)' - alpha)/alpha).^2))/besseli(0, 2.5);
    scale = 2/(sum(w)*fullScale);

    [xu, iu] = unique(xs);
    xg = linspace(xu(1), xu(end), min(1500, numel(xu)));
    pick = sweeps(iu(interp1(xu, 1:numel(xu), xg, 'nearest')));
    image = zeros(nb, numel(pick), 'single');
    for c = 1:500:numel(pick)
        cols = c:min(c + 499, numel(pick));
        X = double(S.signal(:, pick(cols)));
        X = X - mean(X, 1);
        F = fft(X.*w, [], 1);
        image(:, cols) = single(20*log10(abs(F(1:nb, :))*scale + 1e-15));
    end
end
