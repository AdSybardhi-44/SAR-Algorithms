function actuator_csv_to_mat(csv_file, mat_file, radar_timestamps)
%ACTUATOR_CSV_TO_MAT  Stage positions of the stage moves: CSV -> MAT file.
%   ACTUATOR_CSV_TO_MAT(CSV_FILE, MAT_FILE, RADAR_TIMESTAMPS) reads the stage
%   position log CSV_FILE that actuator_logger.m wrote during the measurement
%   (Results\actuator_<date>_<time>.csv) and saves in MAT_FILE the positions
%   of the stage moves during the radar recording (RADAR_TIMESTAMPS, POSIX s
%   UTC), from the last reading before the stage started moving to the
%   reading where it stopped:
%
%       actuator_timestamps  [1 x N] time of every position, POSIX s UTC: the
%                            clock of radar_timestamps
%       actuator_position    [1 x N] stage position along the rail (m) in the
%                            coordinates of the Linear Actuator App (0 where
%                            Create stepper was pressed, negative = left)
%
%   actuator_timestamps(1) is the last reading before the stage started
%   moving (it stood at actuator_position(1)), then come the readings while
%   it moved, and actuator_timestamps(end) is the reading where it stopped.
%   The other readings while it stood still are left out; with several
%   moves, each one starts with its last reading at rest, so over a pause
%   interp1 keeps the position where the stage stood. The moves in the log
%   are listed in the Command Window.
%
%       load("radar_data.mat")
%       actuator_csv_to_mat("actuator_data.csv", "actuator_data.mat", radar_timestamps)
%       load("actuator_data.mat")
%       x = interp1(actuator_timestamps, actuator_position, radar_timestamps);
%
%   gives the stage position of every sweep: NaN before the first reading
%   and after the stop. interp1 is linear between two readings; between the
%   reading at rest and the first reading while moving the stage stood
%   still until shortly before the second one, so there the positions are
%   off by up to the distance between the two readings.
%
%   ACTUATOR_CSV_TO_MAT(CSV_FILE, MAT_FILE) keeps every move in the log; a
%   log often also holds moves outside the recording (to the start position,
%   back after the scan). Without arguments it reads actuator_data.csv and
%   writes actuator_data.mat.
%
%   A move consists of the readings the app takes while the stage moves
%   (about every 0.25 s, and when it stops) and the position where STOP
%   halted it. A move that ended without a final reading (limit switch,
%   timeout, abort of a plan) ends with the first reading of the stage
%   standing still, up to about 0.5 s after it stopped (a warning names it).
%   If the log holds no reading before a move in its coordinate segment,
%   the move starts with its first reading while moving. Positions of
%   different coordinate segments (the position counter set again by
%   Create stepper, Set home here or Auto home) are never mixed.
%
%   See also actuator_logger, Stripmap_SAR_TDBP.

    arguments
        csv_file (1,1) string = "actuator_data.csv"
        mat_file (1,1) string = "actuator_data.mat"
        radar_timestamps double {mustBeFinite} = []
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

    rows = [moves(keep).rows];
    [t, first] = unique(T.timestamp_s(rows));
    x = T.position_mm(rows)/1e3;
    actuator_timestamps = reshape(t, 1, []);
    actuator_position = reshape(x(first), 1, []);
    save(mat_file, "actuator_timestamps", "actuator_position", "-v7");

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
