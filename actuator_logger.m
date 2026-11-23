function varargout = actuator_logger(command, folder)
%ACTUATOR_LOGGER  Log the position of the linear stage with UTC time stamps.
%   Logs where the linear stage of the Linear Actuator App (OTA Hub) is, and
%   when, so that every sweep of a radar recording (radar_gui.py) can be
%   given a stage position afterwards. Run it in the MATLAB that OTA Hub
%   opens for the app - only that MATLAB can read the stage:
%
%       addpath('C:\Users\kryeziuarber\Desktop\SAR-Algorithms')
%       actuator_logger                   % start: Results\actuator_<time>.csv
%       actuator_logger status            % what it logs
%       actuator_logger stop              % stop; the file is complete
%       actuator_logger('start', FOLDER)  % write the file into FOLDER
%       file = actuator_logger(...)       % the file of the log
%
%   Start it before or after Connect / Create stepper / Start: it follows
%   the stage of the open app, also when the app is closed and opened
%   again. The app works as before; the logger only listens:
%     move   While the stage moves (Jog, Go to, Go home, Auto home, the
%            moves of a plan) the app reads the position about every
%            0.25 s and when the move ends. Every one of these readings is
%            logged; the logger sends nothing to the Arduino during a move.
%     halt   STOP (or Esc) halts the stage at once; the position where it
%            stopped is logged.
%     idle   While the app is idle the logger reads the position itself,
%            every 0.5 s - only when MATLAB is not executing anything else,
%            so its reading never mixes with a command of the app.
%   Every row is written to the file at once (at the latest 0.1 s later),
%   so the file is complete even when MATLAB is closed without
%   actuator_logger stop.
%
%   FILE  actuator_<yyyyMMdd_HHmmss>.csv, one row per reading or event:
%     timestamp_s  POSIX seconds UTC of the PC clock when the reading
%                  arrived: the clock of radar_timestamps in the radar
%                  recordings (radar_gui.py sets the radar clock to the PC
%                  clock when a recording starts; it stays within a few ms).
%     position_mm  position in the coordinates of the app (the mm it
%                  shows); NaN in an event row
%     source       move, halt, idle (see above) or event
%     segment      coordinate system of the positions. It changes when the
%                  position counter is set again (Create stepper - the
%                  counter starts at 0 where the stage is -, Set home here,
%                  Auto home) or another stage is followed: positions of
%                  different segments must not be mixed.
%     utc          timestamp_s as text
%     note         the message of an event, or details
%   An event row starting with "Warning:" marks a move that ended without
%   a final reading (limit switch, timeout, abort of a plan): the position
%   is uncertain between the two times it names.
%
%   ACCURACY  Checked with the radar on the stage: the logged positions
%   match the motion the radar sees within about 20 ms (0.05 mm at
%   2.5 mm/s). After the stage reverses its direction the carriage lags the
%   logged position at the start of the move (backlash: the lag fell from
%   about 0.3 s to 0.05 s within 5 s): start a scan with some travel in its
%   direction before the part you use.
%
%   STAGE POSITION OF EVERY RADAR SWEEP
%       A = readtable('Results\actuator_20261120_141500.csv');
%       R = load('Results\acquisition_20261120_141530.mat');
%       t = R.radar_timestamps;
%       k = ~isnan(A.position_mm) & A.timestamp_s >= t(1) - 10 & ...
%           A.timestamp_s <= t(end) + 10;
%       unique(A.segment(k))            % must be one segment
%       x = interp1(A.timestamp_s(k), A.position_mm(k), t);  % mm, per sweep
%   x is NaN for the sweeps outside the logged time.
%
%   See also LinearActuatorApp, laapp.StageDriver.

    if nargin < 1 || isempty(command)
        command = 'start';
    end
    command = lower(strtrim(char(command)));
    tmr = findLogger();
    switch command
        case 'start'
            if ~isempty(tmr) && isRunning(tmr)
                file = tmr.UserData.file;
                fprintf('actuator_logger is running already: %s\n', file);
                fprintf('actuator_logger status shows what it logs, actuator_logger stop ends it.\n');
            else
                if ~isempty(tmr)
                    stopLogger(tmr, false);
                end
                if nargin < 2 || isempty(folder)
                    folder = fullfile(fileparts(mfilename('fullpath')), 'Results');
                end
                file = startLogger(char(folder));
            end
        case 'stop'
            file = stopLogger(tmr, true);
        case 'status'
            file = printStatus(tmr);
        otherwise
            error('actuator_logger:command', ...
                  'The command is start, stop or status, not "%s".', command);
    end
    if nargout > 0
        varargout{1} = file;
    end
end

% =========================================================================
% Start, stop, status
% =========================================================================

function tag = loggerTag()
    tag = 'actuator_logger';
end

function tmr = findLogger()
    tmr = timerfindall('Tag', loggerTag());
    if numel(tmr) > 1
        for k = 2:numel(tmr)
            stopLogger(tmr(k), false);
        end
        tmr = tmr(1);
    end
end

function tf = isRunning(tmr)
    tf = isvalid(tmr) && strcmp(char(tmr.Running), 'on');
end

function file = startLogger(folder)
    if ~isfolder(folder)
        mkdir(folder);
    end
    file = newFile(folder);
    fid = fopen(file, 'w');
    if fid < 0
        error('actuator_logger:file', 'The log cannot be written to %s.', file);
    end
    fprintf(fid, 'timestamp_s,position_mm,source,segment,utc,note\n');
    fclose(fid);

    s = initialState(file);
    s = addRow(s, utcNow(), NaN, 'event', sprintf('Logger started (MATLAB %s).', version));
    s = writeRows(s);
    tmr = timer('Name', 'actuator_logger', 'Tag', loggerTag(), ...
                'ExecutionMode', 'fixedSpacing', 'Period', 0.1, 'BusyMode', 'drop', ...
                'ObjectVisibility', 'off');
    tmr.UserData = s;
    tmr.TimerFcn = @(t, ~) onTimer(t);
    tmr.StopFcn = @(t, ~) onTimerStopped(t);
    start(tmr);
    fprintf('actuator_logger: logging the stage position to\n  %s\n', file);
    fprintf(['It follows the Linear Actuator App (Connect, Create stepper, Start). ' ...
             'actuator_logger stop ends it.\n']);
end

function file = newFile(folder)
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    file = fullfile(folder, ['actuator_' stamp '.csv']);
    k = 0;
    while isfile(file)
        k = k + 1;
        file = fullfile(folder, sprintf('actuator_%s_%d.csv', stamp, k));
    end
end

function file = stopLogger(tmr, verbose)
    file = '';
    if isempty(tmr) || ~isvalid(tmr)
        if verbose
            fprintf('actuator_logger is not running.\n');
        end
        return
    end
    s = tmr.UserData;
    s.stopRequested = true;
    tmr.UserData = s;
    if isRunning(tmr)
        stop(tmr);
    end
    s = finish(tmr.UserData);
    tmr.UserData = s;
    % finish() did what the StopFcn would do when the timer is deleted
    tmr.StopFcn = '';
    delete(tmr);
    file = s.file;
    if verbose
        c = s.counts;
        fprintf(['actuator_logger stopped: %d rows (positions: %d move, %d halt, %d idle) ' ...
                 'in\n  %s\n'], s.written, c.move, c.halt, c.idle, s.file);
    end
    if ~isempty(s.pending)
        warning('actuator_logger:file', ['%d rows could not be written to %s ' ...
                '(open in another program?).'], size(s.pending, 1), s.file);
    end
end

function s = finish(s)
    if s.finished
        s = writeRows(s);
        return
    end
    s = unhook(s);
    s = addRow(s, utcNow(), NaN, 'event', 'Logger stopped.');
    s = writeRows(s);
    s.finished = true;
end

function onTimerStopped(tmr)
    % Only an unexpected stop; actuator_logger stop finishes itself.
    try
        if ~isvalid(tmr)
            return
        end
        s = tmr.UserData;
        if ~isstruct(s) || s.stopRequested
            return
        end
        s = finish(s);
        tmr.UserData = s;
        warning('actuator_logger:stopped', ['actuator_logger stopped unexpectedly; %s is ' ...
                'complete up to now. Start it again with actuator_logger.'], s.file);
    catch err
        warning('actuator_logger:stopped', 'actuator_logger stopped: %s', err.message);
    end
end

function file = printStatus(tmr)
    file = '';
    if isempty(tmr) || ~isvalid(tmr)
        fprintf('actuator_logger is not running (start it with actuator_logger).\n');
        return
    end
    s = tmr.UserData;
    file = s.file;
    if isRunning(tmr)
        fprintf('actuator_logger is running: %s\n', s.file);
    else
        fprintf('actuator_logger is STOPPED: %s\n', s.file);
    end
    c = s.counts;
    fprintf('  rows written %d (%d waiting); positions: %d move, %d halt, %d idle; %d events\n', ...
            s.written, size(s.pending, 1), c.move, c.halt, c.idle, c.event);
    fprintf('  stage: %s\n', s.state);
    fprintf('  following the readings of the app: %s; segment %d\n', ...
            yesNo(~isempty(s.wrapPos)), s.segment);
    if isfinite(s.lastT)
        fprintf('  last position %.4f mm (%s) at %s UTC, %.1f s ago\n', s.lastX, ...
                s.lastSource, utcText(s.lastT), utcNow() - s.lastT);
    else
        fprintf('  no position logged in this segment yet\n');
    end
end

function s = initialState(file)
    s = struct('file', file, 'pending', {cell(0, 5)}, 'written', 0, ...
        'counts', struct('move', 0, 'halt', 0, 'idle', 0, 'event', 0), ...
        'segment', 0, 'app', [], 'drv', [], ...
        'origPos', [], 'wrapPos', [], 'origLog', [], 'wrapLog', [], ...
        'state', '', 'readSoon', true, 'lastIdleRead', -Inf, 'readFailUntil', -Inf, ...
        'lastT', NaN, 'lastX', NaN, 'lastSource', '', ...
        'writeFailSince', NaN, 'writeWarned', false, ...
        'stopRequested', false, 'finished', false, 'lastErrorT', -Inf);
end

% =========================================================================
% Timer: follow the app, read the position while it is idle, write rows
% =========================================================================

function onTimer(tmr)
    try
        tick(tmr);
    catch err
        % An error would stop the timer: log it (at most every 10 s) and go on.
        try
            s = tmr.UserData;
            t = utcNow();
            if t - s.lastErrorT >= 10
                s.lastErrorT = t;
                s = addRow(s, t, NaN, 'event', ['Logger error: ' err.message]);
                tmr.UserData = s;
            end
        catch
        end
    end
end

function tick(tmr)
    s = tmr.UserData;
    [app, d] = findStage(s.app);

    if ~sameObject(d, s.drv)
        hadStage = ~isempty(s.drv);
        s = unhook(s);
        if ~isempty(d)
            s.segment = s.segment + 1;
            s.lastT = NaN;
            s.lastX = NaN;
            s.lastSource = '';
            s = hook(s, tmr, d);
            s = addRow(s, utcNow(), NaN, 'event', sprintf( ...
                'Following the stage of the Linear Actuator App (%s): segment %d.', ...
                d.boardName(), s.segment));
            s.readSoon = true;
        elseif hadStage
            s = addRow(s, utcNow(), NaN, 'event', ...
                       'The stage is gone (app closed?); waiting for it.');
        end
    elseif ~isempty(d)
        s = rehook(s, tmr, d);
    end
    s.app = app;

    st = stageState(app, d);
    if ~strcmp(st, s.state)
        s.state = st;
        s = addRow(s, utcNow(), NaN, 'event', ['Stage: ' st]);
    end

    busy = isempty(app) || appBusy(app);
    if busy
        % read as soon as the app is idle again: a move may have ended
        s.readSoon = true;
    end
    t = utcNow();
    doRead = ~isempty(d) && ~busy && d.Connected && d.StepperReady ...
        && t >= s.readFailUntil && (s.readSoon || t - s.lastIdleRead >= 0.5) ...
        && matlabIdle();
    tmr.UserData = s;
    if doRead
        s = readIdle(tmr, d);
    end
    s = writeRows(s);
    tmr.UserData = s;
end

function s = readIdle(tmr, d)
    err = [];
    try
        x = double(d.position());
    catch err
        x = NaN;
    end
    t = utcNow();
    % the hooks may have added rows during the reading
    s = tmr.UserData;
    if ~isempty(err)
        s = addRow(s, t, NaN, 'event', ['Reading the position failed: ' err.message]);
        s.readFailUntil = t + 5;
        return
    end
    s.lastIdleRead = t;
    if ~isfinite(x)
        return
    end
    % A logged move ends with a reading (or STOP's halt) where the stage
    % stands now; a difference means a move ended without one.
    lastT = s.lastT;
    lastX = s.lastX;
    lastSource = s.lastSource;
    s = addRow(s, t, x, 'idle', '');
    s.readSoon = false;
    if isfinite(lastX) && abs(x - lastX) > stepTolerance(d)
        s = addRow(s, utcNow(), NaN, 'event', sprintf(['Warning: the stage stands at %.4f mm, ' ...
            '%.4f mm from the last logged position (%.4f mm, %s, %s UTC): a move ended ' ...
            'without a final reading (limit switch, timeout or abort of a plan). The ' ...
            'position is uncertain between %s and %s UTC.'], x, x - lastX, lastX, ...
            lastSource, utcText(lastT), utcText(lastT), utcText(t)));
    end
end

function tol = stepTolerance(d)
    tol = 0.002;
    try
        tol = 1.5/d.stepsPerMm();
    catch
    end
end

function tf = matlabIdle()
    % True when this timer callback did not interrupt other code: MATLAB
    % was idle, so no command of the app can be half-way through.
    st = dbstack;
    names = {st.name};
    k = find(strcmp(names, 'timercb'), 1, 'last');
    tf = ~isempty(k) && k == numel(names);
end

function tf = appBusy(app)
    try
        tf = logical(app.Busy);
    catch
        tf = true;
    end
end

function [app, d] = findStage(prev)
    app = [];
    d = [];
    if isLiveApp(prev)
        app = prev;
    else
        figs = allchild(groot);
        for k = 1:numel(figs)
            f = figs(k);
            try
                if isvalid(f) && strcmp(f.Tag, 'laApp') && isappdata(f, 'laApp')
                    a = getappdata(f, 'laApp');
                    if isLiveApp(a)
                        app = a;
                        break
                    end
                end
            catch
            end
        end
    end
    if isempty(app)
        return
    end
    try
        d = app.Driver;
    catch
        d = [];
    end
    if ~(isa(d, 'laapp.StageDriver') && isscalar(d) && isvalid(d))
        d = [];
    end
end

function tf = isLiveApp(a)
    tf = false;
    try
        tf = isa(a, 'LinearActuatorApp') && isscalar(a) && isvalid(a) && ...
             ~isempty(a.Fig) && isvalid(a.Fig) && isappdata(a.Fig, 'laApp');
    catch
    end
end

function tf = sameObject(a, b)
    if isempty(a) || isempty(b)
        tf = isempty(a) && isempty(b);
    else
        tf = isvalid(a) && isvalid(b) && a == b;
    end
end

function st = stageState(app, d)
    if isempty(app)
        st = 'no Linear Actuator App is open';
        return
    end
    if isempty(d)
        st = 'the app has no stage driver';
        return
    end
    try
        ms = d.AppliedMicrostep;
        if ~isfinite(ms)
            ms = d.Microstep;
        end
        st = sprintf(['%s, port %s, connected %s, stepper %s, started %s, homed %s, ' ...
                      'microstep %d (%g steps/mm), speed %g mm/s, acceleration %g mm/s^2'], ...
                     d.boardName(), textOr(d.Port, '-'), yesNo(d.Connected), ...
                     yesNo(d.StepperReady), yesNo(d.Started), yesNo(d.Homed), ms, ...
                     ms/d.LeadMm, d.SpeedMms, d.AccelMms2);
    catch err
        st = ['unknown (' err.message ')'];
    end
end

% =========================================================================
% Hooks: the readings and the messages of the stage driver
% =========================================================================

function s = hook(s, tmr, d)
    s.drv = d;
    s = rehook(s, tmr, d);
end

function s = rehook(s, tmr, d)
    % Wrap the functions the app gave the driver; when the app installs new
    % ones (or removes them) follow it. Never install one the app has not.
    % (A wrapper left by a deleted logger only passes the calls on.)
    f = d.PositionFcn;
    if ~isequal(f, s.wrapPos)
        s.origPos = f;
        s.wrapPos = [];
        if ~isempty(f)
            s.wrapPos = @(mm) onPosition(tmr, f, mm);
            d.PositionFcn = s.wrapPos;
        end
    end
    f = d.Log;
    if ~isequal(f, s.wrapLog)
        s.origLog = f;
        s.wrapLog = [];
        if ~isempty(f)
            s.wrapLog = @(msg) onLog(tmr, d, f, msg);
            d.Log = s.wrapLog;
        end
    end
end

function s = unhook(s)
    d = s.drv;
    if ~isempty(d) && isvalid(d)
        try
            if ~isempty(s.wrapPos) && isequal(d.PositionFcn, s.wrapPos)
                d.PositionFcn = s.origPos;
            end
            if ~isempty(s.wrapLog) && isequal(d.Log, s.wrapLog)
                d.Log = s.origLog;
            end
        catch
        end
    end
    s.drv = [];
    s.origPos = [];
    s.wrapPos = [];
    s.origLog = [];
    s.wrapLog = [];
end

function onPosition(tmr, orig, mm)
    % The app read the position during a move (StageDriver.pollMove). The
    % row is written at once: timers may not run while the app moves.
    t = utcNow();
    try
        if isRunning(tmr)
            s = tmr.UserData;
            s = addRow(s, t, double(mm), 'move', '');
            s = writeRows(s);
            tmr.UserData = s;
        end
    catch
    end
    orig(mm);
end

function onLog(tmr, d, orig, msg)
    % A message of the stage driver: STOP's halt, or a new coordinate system.
    t = utcNow();
    try
        if isRunning(tmr)
            s = tmr.UserData;
            text = char(msg);
            if startsWith(text, 'Halted at ')
                % StageDriver.halt has just set LastTarget to the halted counter
                s = addRow(s, t, double(d.LastTarget), 'halt', text);
            else
                if newCoordinates(text)
                    s.segment = s.segment + 1;
                    s.lastT = NaN;
                    s.lastX = NaN;
                    s.lastSource = '';
                end
                s = addRow(s, t, NaN, 'event', text);
            end
            s = writeRows(s);
            tmr.UserData = s;
        end
    catch
    end
    orig(msg);
end

function tf = newCoordinates(text)
    % The messages of laapp.StageDriver after the counter was set again.
    tf = contains(text, 'The counter starts at 0 here') || ...
         startsWith(text, 'Home set here') || ...
         startsWith(text, 'Min switch reached') || ...
         startsWith(text, 'Auto home done');
end

% =========================================================================
% Rows and the CSV file
% =========================================================================

function s = addRow(s, t, x, source, note)
    s.pending(end+1, :) = {t, x, source, s.segment, note};
    s.counts.(source) = s.counts.(source) + 1;
    if isfinite(x)
        s.lastT = t;
        s.lastX = x;
        s.lastSource = source;
    end
end

function s = writeRows(s)
    n = size(s.pending, 1);
    if n == 0
        return
    end
    text = formatRows(s.pending);
    fid = fopen(s.file, 'a');
    if fid < 0
        % e.g. open in Excel: keep the rows and try again
        t = utcNow();
        if isnan(s.writeFailSince)
            s.writeFailSince = t;
        end
        if ~s.writeWarned && t - s.writeFailSince > 2
            s.writeWarned = true;
            warning('actuator_logger:file', ['%s cannot be written (open in another ' ...
                    'program?). The rows are kept and written when it can be written again.'], ...
                    s.file);
        end
        return
    end
    fprintf(fid, '%s', text);
    fclose(fid);
    s.pending = cell(0, 5);
    s.written = s.written + n;
    if s.writeWarned
        fprintf('actuator_logger: %s is written again.\n', s.file);
    end
    s.writeFailSince = NaN;
    s.writeWarned = false;
end

function text = formatRows(rows)
    n = size(rows, 1);
    t = [rows{:, 1}];
    x = [rows{:, 2}];
    utc = utcText(t);
    if n == 1
        utc = {utc};
    end
    lines = cell(1, n);
    for k = 1:n
        lines{k} = sprintf('%.6f,%.6f,%s,%d,%s,%s\n', t(k), x(k), rows{k, 3}, rows{k, 4}, ...
                           utc{k}, csvText(rows{k, 5}));
    end
    text = [lines{:}];
end

function q = csvText(note)
    note = regexprep(char(note), '[\r\n]+', ' ');
    q = ['"' strrep(note, '"', '""') '"'];
end

% =========================================================================
% Helpers
% =========================================================================

function t = utcNow()
    % POSIX seconds UTC of the precise Windows system time, the clock of
    % radar_setup.utc_now (an unzoned datetime('now') would be local time).
    t = posixtime(datetime('now', 'TimeZone', 'UTC'));
end

function c = utcText(t)
    % Text of POSIX time(s): a char for one, a cellstr for several.
    d = datetime(t, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC', ...
                 'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSS''Z''');
    c = cellstr(d);
    if isscalar(t)
        c = c{1};
    end
end

function s = yesNo(tf)
    if tf
        s = 'yes';
    else
        s = 'no';
    end
end

function s = textOr(s, default)
    s = char(s);
    if isempty(s)
        s = default;
    end
end
