function actuator_csv_to_mat(csv_file, mat_file, radar_timestamps, options)
%ACTUATOR_CSV_TO_MAT  Stage positions of the stage moves: CSV -> MAT file.
%   ACTUATOR_CSV_TO_MAT(CSV_FILE, MAT_FILE, RADAR_TIMESTAMPS) reads the stage
%   position log CSV_FILE that actuator_logger.m wrote during the measurement
%   (Results\actuator_<date>_<time>.csv) and saves in MAT_FILE the constant-
%   velocity cruise portion of the stage scan during the radar recording
%   (RADAR_TIMESTAMPS, POSIX s UTC), trimming quasi-static idle endpoints,
%   startup acceleration, shutdown deceleration, and pauses:
%
%       actuator_timestamps  [1 x N] time of every position, POSIX s UTC: the
%                            clock of radar_timestamps
%       actuator_position    [1 x N] stage position along the rail (m) in the
%                            coordinates of the Linear Actuator App (0 where
%                            Create stepper was pressed, negative = left)
%       actuator_velocity    [1 x 1] fitted constant cruise velocity (m/s)
%
%   With ConstantVelocity=true (default), only the longest constant-velocity
%   segment overlapping RADAR_TIMESTAMPS is kept (within VelocityTolerance,
%   default 0.15 = +/-15% of the median cruise velocity). Single-poll USB
%   timestamp jitter spikes inside the move are preserved as one contiguous
%   segment using a 3-interval chord velocity check, while genuine ramps,
%   stalls, or stationary pauses are trimmed out.
%
%       load("radar_data.mat")
%       actuator_csv_to_mat("actuator_data.csv", "actuator_data.mat", radar_timestamps)
%       load("actuator_data.mat")
%       mask = actuator_timestamps(1) <= radar_timestamps & ...
%              radar_timestamps <= actuator_timestamps(end);
%       x = interp1(actuator_timestamps, actuator_position, radar_timestamps(mask));
%
%   ACTUATOR_CSV_TO_MAT(..., ConstantVelocity=false) keeps every move during
%   the recording from the last reading at rest before it started to the
%   reading where it stopped. Without RADAR_TIMESTAMPS, all moves in the log
%   are considered. Without arguments it reads actuator_data.csv and writes
%   actuator_data.mat.
%
%   Positions of different coordinate segments (the position counter set
%   again by Create stepper, Set home here or Auto home) are never mixed.
%
%   See also actuator_logger, Stripmap_SAR_TDBP.

    arguments
        csv_file (1,1) string = "actuator_data.csv"
        mat_file (1,1) string = "actuator_data.mat"
        radar_timestamps double {mustBeFinite} = []
        options.ConstantVelocity (1,1) logical = true
        options.VelocityTolerance (1,1) double {mustBePositive, mustBeLessThan(options.VelocityTolerance, 1)} = 0.15
    end

    if ~isempty(radar_timestamps) && min(radar_timestamps(:)) < 1e9
        error("actuator_csv_to_mat:time", ...
              "RADAR_TIMESTAMPS must be the radar_timestamps of the recording (POSIX s UTC).");
    end

    T = readLog(csv_file);
    moves = findMoves(T);
    if isempty(moves)
        error("actuator_csv_to_mat:moves", "The stage did not move in %s.", csv_file);
    end

    if isempty(radar_timestamps)
        keep = true(1, numel(moves));
    else
        t0 = min(radar_timestamps(:));
        t1 = max(radar_timestamps(:));
        keep = [moves.stop] >= t0 & [moves.start] <= t1;
    end
    printMoves(csv_file, moves, keep, radar_timestamps);

    if ~any(keep)
        error("actuator_csv_to_mat:moves", ['The stage did not move during the radar recording ' ...
              '(%s to %s): is %s the stage log of this measurement?'], clockText(t0), ...
              clockText(t1, "HH:mm:ss"), csv_file);
    end
    if ~isscalar(unique([moves(keep).segment]))
        if isempty(radar_timestamps)
            error("actuator_csv_to_mat:segment", ['The stage moved in several coordinate segments ' ...
                  '(the position counter was set again by Create stepper, Set home here or Auto ' ...
                  'home): give the radar_timestamps of the recording to keep its moves, e.g. ' ...
                  'actuator_csv_to_mat("%s", "%s", radar_timestamps)'], csv_file, mat_file);
        end
        error("actuator_csv_to_mat:segment", ['The position counter was set again during the ' ...
              'radar recording (Create stepper, Set home here or Auto home): the positions ' ...
              'before and after are not comparable.']);
    end

    if options.ConstantVelocity
        seg = selectConstantVelocity(T, moves, keep, radar_timestamps, options.VelocityTolerance);
        actuator_timestamps = seg.t;
        actuator_position = seg.x;
        actuator_velocity = seg.v_fit;
        save(mat_file, "actuator_timestamps", "actuator_position", "actuator_velocity", "-v7");

        fprintf("Saved %s (constant velocity): %d positions, from %s at %.3f mm to %s at %.3f mm\n", ...
                mat_file, numel(actuator_timestamps), ...
                clockText(actuator_timestamps(1), "HH:mm:ss.SSS"), 1e3*actuator_position(1), ...
                clockText(actuator_timestamps(end), "HH:mm:ss.SSS"), 1e3*actuator_position(end));
        fprintf("  cruise velocity %+.3f mm/s (tol +/-%.0f%%, interval std %.3f mm/s; linear-fit residual RMS %.3f mm, max %.3f mm)\n", ...
                1e3*seg.v_fit, 100*options.VelocityTolerance, 1e3*seg.v_std, ...
                seg.res_rms_mm, seg.res_max_mm);
        fprintf("  trimmed quasi-static/ramp ends of move %d: %.2f s (%.3f mm) at start, %.2f s (%.3f mm) at stop\n", ...
                seg.move_index, seg.trim_start_s, 1e3*seg.trim_start_m, ...
                seg.trim_stop_s, 1e3*seg.trim_stop_m);
        if seg.n_candidates > 1
            warning("actuator_csv_to_mat:multipleSegments", ...
                    "Found %d constant-velocity segments during the recording; kept the longest one (move %d, %.1f mm).", ...
                    seg.n_candidates, seg.move_index, 1e3*abs(actuator_position(end) - actuator_position(1)));
        end
    else
        rows = [moves(keep).rows];
        [t, first] = unique(T.timestamp_s(rows));
        x = T.position_mm(rows)/1e3;
        actuator_timestamps = reshape(t, 1, []);
        actuator_position = reshape(x(first), 1, []);
        if isscalar(find(keep)) && numel(actuator_timestamps) >= 2
            actuator_velocity = fitVelocity(actuator_timestamps, actuator_position);
        else
            actuator_velocity = NaN;
        end
        save(mat_file, "actuator_timestamps", "actuator_position", "actuator_velocity", "-v7");

        fprintf("Saved %s: %d positions, from %s at %.3f mm to %s at %.3f mm\n", ...
                mat_file, numel(actuator_timestamps), clockText(actuator_timestamps(1), "HH:mm:ss.SSS"), ...
                1e3*actuator_position(1), clockText(actuator_timestamps(end), "HH:mm:ss.SSS"), ...
                1e3*actuator_position(end));
        for k = find(keep & [moves.open_end])
            m = moves(k);
            warning("actuator_csv_to_mat:stop", ['Move %d ended without a final reading (limit ' ...
                    'switch, timeout or abort of a plan): it stopped at %.3f mm at some time between ' ...
                    '%s and %s, the time saved for its stop.'], k, m.to, ...
                    clockText(T.timestamp_s(m.rows(end - 1)), "HH:mm:ss.SSS"), ...
                    clockText(m.stop, "HH:mm:ss.SSS"));
        end
    end
end

% -------------------------------------------------------------------------
function T = readLog(csv_file)
    header = "timestamp_s,position_mm,source,segment,utc,note";
    fid = fopen(csv_file);
    if fid < 0
        error("actuator_csv_to_mat:file", "Cannot open %s.", csv_file);
    end
    first_line = fgetl(fid);
    fclose(fid);
    if ~ischar(first_line)
        first_line = '';
    end
    if strip(string(first_line)) ~= header
        error("actuator_csv_to_mat:file", ['%s is not a stage position log of actuator_logger.m ' ...
              '(first line: "%s"); copy the CSV without saving it in Excel.'], csv_file, first_line);
    end
    opts = delimitedTextImportOptions("Delimiter", ",", "DataLines", 2, ...
        "VariableNames", split(header, ",")', ...
        "VariableTypes", ["double", "double", "string", "double", "string", "string"]);
    T = readtable(csv_file, opts);
end

function moves = findMoves(T)
    % The moves: runs of consecutive readings that show the stage moving,
    % each with the reading before it (the stage at rest), per coordinate
    % segment, in time order.
    moves = struct("rows", {}, "segment", {}, "start", {}, "stop", {}, ...
                   "from", {}, "to", {}, "open_end", {});
    readings = find(ismember(T.source, ["move", "idle", "halt"]) & isfinite(T.position_mm));
    for s = unique(T.segment(readings))'
        r = readings(T.segment(readings) == s);
        [~, order] = sort(T.timestamp_s(r));
        r = r(order);
        % A reading of the app during a move, or a STOP or idle reading of
        % the stage no longer where it was read last (halted during a move,
        % or a move without a final reading); the other readings show the
        % stage standing still and separate the moves.
        moving = T.source(r) == "move" | [false; diff(T.position_mm(r)) ~= 0];
        edges = diff([false; moving; false]);
        first = find(edges == 1);
        last = find(edges == -1) - 1;
        for k = 1:numel(first)
            run = r(max(first(k) - 1, 1):last(k));
            x = T.position_mm(run);
            if max(x) == min(x)
                continue    % did not change the position
            end
            moves(end + 1) = struct("rows", run', "segment", s, ...
                "start", T.timestamp_s(r(first(k))), "stop", T.timestamp_s(run(end)), ...
                "from", x(1), "to", x(end), "open_end", T.source(run(end)) == "idle"); %#ok<AGROW>
        end
    end
    [~, order] = sort([moves.start]);
    moves = moves(order);
end

function printMoves(csv_file, moves, keep, radar_timestamps)
    fprintf("%s: the stage moved %d time(s) (local time, %s):\n", csv_file, numel(moves), ...
            clockText(moves(1).start, "dd.MM.yyyy"));
    filtered = ~isempty(radar_timestamps);
    for k = 1:numel(moves)
        m = moves(k);
        mark = " ";
        if filtered && keep(k)
            mark = ">";
        end
        fprintf("  %s %2d  %s - %s  %7.1f s  %9.3f -> %9.3f mm  (segment %d)\n", mark, k, ...
                clockText(m.start, "HH:mm:ss.SSS"), clockText(m.stop, "HH:mm:ss.SSS"), ...
                m.stop - m.start, m.from, m.to, m.segment);
    end
    if filtered
        fprintf("  radar recording %s - %s (> = moves during it)\n", ...
                clockText(min(radar_timestamps(:)), "HH:mm:ss.SSS"), ...
                clockText(max(radar_timestamps(:)), "HH:mm:ss.SSS"));
    end
end

function s = clockText(t, format)
    if nargin < 2
        format = "dd.MM.yyyy HH:mm:ss";
    end
    s = string(datetime(t, "ConvertFrom", "posixtime", "TimeZone", "local", "Format", format));
end

function best = selectConstantVelocity(T, moves, keep, radar_timestamps, vel_tol)
    % Extract constant-velocity cruise segments across kept moves and return
    % the one with the longest constant-velocity travel during the recording.
    has_rec = ~isempty(radar_timestamps);
    if has_rec
        t0 = min(radar_timestamps(:));
        t1 = max(radar_timestamps(:));
    end

    candidates = struct("t", {}, "x", {}, "v_fit", {}, "v_std", {}, ...
                        "res_rms_mm", {}, "res_max_mm", {}, "travel_rec", {}, ...
                        "move_index", {}, "trim_start_s", {}, "trim_start_m", {}, ...
                        "trim_stop_s", {}, "trim_stop_m", {}, "n_candidates", {});

    for k = find(keep)
        m = moves(k);
        [t, first] = unique(T.timestamp_s(m.rows));
        r_unique = m.rows(first);
        x = T.position_mm(r_unique) / 1e3;
        src = T.source(r_unique);
        if numel(t) < 2
            continue
        end

        dt = diff(t);
        dx = diff(x);
        v_raw = dx ./ dt;

        % Readings with source == "idle" were polled while the stage stood
        % still (before the move started, or after an unlogged stop).
        non_idle = src(1:end-1) ~= "idle" & src(2:end) ~= "idle";
        if has_rec
            in_rec = t(2:end) >= t0 & t(1:end-1) <= t1;
        else
            in_rec = true(size(dt));
        end

        ref_mask = non_idle & in_rec & isfinite(v_raw) & abs(v_raw) > 5e-5;
        if ~any(ref_mask)
            ref_mask = non_idle & isfinite(v_raw) & abs(v_raw) > 5e-5;
        end
        if ~any(ref_mask)
            continue
        end

        v_ref = weightedMedian(v_raw(ref_mask), abs(dx(ref_mask)));
        dir = sign(v_ref);

        % 3-interval chord velocity cancels single-poll PC/USB timestamp
        % jitter (down-spike + up-spike pairs) without bridging idle polls.
        n_int = numel(dt);
        v_chord = nan(n_int, 1);
        for j = 1:n_int
            if ~non_idle(j)
                continue
            end
            j_lo = j;
            if j > 1 && non_idle(j - 1)
                j_lo = j - 1;
            end
            j_hi = j;
            if j < n_int && non_idle(j + 1)
                j_hi = j + 1;
            end
            v_chord(j) = (x(j_hi + 1) - x(j_lo)) / (t(j_hi + 1) - t(j_lo));
        end

        rel_raw = abs(v_raw - v_ref) / abs(v_ref);
        rel_chord = abs(v_chord - v_ref) / abs(v_ref);

        in_cruise = non_idle & (dx * dir > 0) & (rel_raw <= vel_tol | rel_chord <= vel_tol);
        edges = diff([false; in_cruise; false]);
        run_start = find(edges == 1);
        run_stop = find(edges == -1) - 1;

        for u = 1:numel(run_start)
            a = run_start(u);
            b = run_stop(u);
            % Peel ramp intervals at the boundaries using single-step velocity.
            while a <= b && rel_raw(a) > vel_tol
                a = a + 1;
            end
            while b >= a && rel_raw(b) > vel_tol
                b = b - 1;
            end
            if a > b
                continue
            end

            t_seg = t(a:b+1)';
            x_seg = x(a:b+1)';

            if has_rec
                if t_seg(end) < t0 || t_seg(1) > t1
                    continue
                end
                t_ov0 = max(t_seg(1), t0);
                t_ov1 = min(t_seg(end), t1);
                if t_ov1 <= t_ov0
                    continue
                end
                x_ov = interp1(t_seg, x_seg, [t_ov0, t_ov1], "linear");
                travel_rec = abs(x_ov(2) - x_ov(1));

                % Keep one bracketing reading on each side of [t0, t1] so
                % interp1 covers every cruise sweep without NaNs.
                k0 = find(t_seg <= t0, 1, "last");
                if isempty(k0)
                    k0 = 1;
                end
                k1 = find(t_seg >= t1, 1, "first");
                if isempty(k1)
                    k1 = numel(t_seg);
                end
                if k1 == k0
                    if k1 < numel(t_seg)
                        k1 = k1 + 1;
                    else
                        k0 = k0 - 1;
                    end
                end
                t_seg = t_seg(k0:k1);
                x_seg = x_seg(k0:k1);
            else
                travel_rec = abs(x_seg(end) - x_seg(1));
            end

            [v_fit, res_rms_mm, res_max_mm] = fitVelocity(t_seg, x_seg);
            v_seg = diff(x_seg) ./ diff(t_seg);
            if isscalar(v_seg)
                v_std = 0;
            else
                v_std = std(v_seg, 0);
            end

            candidates(end + 1) = struct( ...
                "t", t_seg, "x", x_seg, ...
                "v_fit", v_fit, "v_std", v_std, ...
                "res_rms_mm", res_rms_mm, "res_max_mm", res_max_mm, ...
                "travel_rec", travel_rec, "move_index", k, ...
                "trim_start_s", max(t_seg(1) - t(1), 0), ...
                "trim_start_m", abs(x_seg(1) - x(1)), ...
                "trim_stop_s", max(t(end) - t_seg(end), 0), ...
                "trim_stop_m", abs(x(end) - x_seg(end)), ...
                "n_candidates", 0); %#ok<AGROW>
        end
    end

    if isempty(candidates)
        error("actuator_csv_to_mat:velocity", ...
              ['No constant-velocity stage motion (within +/-%.0f%%) was found during the ' ...
               'recording. Increase VelocityTolerance or pass ConstantVelocity=false.'], ...
              100 * vel_tol);
    end

    [~, idx] = max([candidates.travel_rec]);
    best = candidates(idx);
    best.n_candidates = numel(candidates);
end

function [v_fit, res_rms_mm, res_max_mm] = fitVelocity(t, x)
    dt_mid = t - mean(t);
    dx_mid = x - mean(x);
    v_fit = sum(dt_mid .* dx_mid) / sum(dt_mid.^2);
    res_mm = 1e3 * (dx_mid - v_fit * dt_mid);
    res_rms_mm = sqrt(mean(res_mm.^2));
    res_max_mm = max(abs(res_mm));
end

function m = weightedMedian(vals, w)
    [vals, order] = sort(vals(:));
    w = w(order);
    csum = cumsum(w) / sum(w);
    m = vals(find(csum >= 0.5, 1, "first"));
end
