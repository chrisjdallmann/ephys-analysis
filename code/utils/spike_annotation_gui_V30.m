function spike_annotation_gui(signal)
% 

% Parameters
fs = 20000; % Sampling rate
windowSec = 2;
windowSize = fs * windowSec;
peakThresh = 1;
t = (0:length(signal)-1)/fs;

% State variables
startIdx = 1;
yLimits = [min(signal), max(signal)];
manualPeaks = []; % [time, amplitude]
peakfinderValues = [];
peakWindow = 0.05;
lowThresh.overlay = 0.5;  % Default value

 % === Shared Variables ===
    defaultStride = 3;
    strideValues = struct('overlay', defaultStride, 'analysis', defaultStride);
    strideValues.overlay = defaultStride;
    strideValues.analysis = defaultStride;


persistent rangeLines %used to visualize where clicked on during manual adding peaks

% Shared variables for overlay
windowMs = 5;
winSamples = round((windowMs / 1000) * fs);
halfWin = floor(winSamples / 2);
tWin = (-halfWin:halfWin) / fs * 1000;

% GUI Layout
screenSize = get(0, 'ScreenSize');
fig = figure('Name','Peak Detector GUI','NumberTitle','off',...
    'WindowButtonDownFcn',@mouseClick,...
    'Position',[1, 100, screenSize(3), round(screenSize(4)*0.6)]);
% ax = axes('Parent',fig);
ax = axes('Parent', fig, ...
    'Units', 'normalized', ...
    'Position', [0.05, 0.25, 0.9, 0.7]);  % [left, bottom, width, height]

xlabel(ax, 'Time (s)'); ylabel(ax, 'Vm (mv)');
            title(ax, 'Raw Signal Trace');
hold on;

% Base Y levels for organization
row_y1 = 10;      % row 1
row_y2 = 50;      % row 2 (above row 1)
row_y3 = 90;      % row 3 (above row 2)

% Navigation Buttons
uicontrol('Style','pushbutton','String','←','Position',[10 row_y1 40 30],...
    'Callback',@(~,~) shiftX(-0.5));
uicontrol('Style','pushbutton','String','→','Position',[60 row_y1 40 30],...
    'Callback',@(~,~) shiftX(0.5));
uicontrol('Style','pushbutton','String','Zoom In X','Position',[110 row_y1 80 30],...
    'Callback',@(~,~) zoomX(0.5));
uicontrol('Style','pushbutton','String','Zoom Out X','Position',[200 row_y1 80 30],...
    'Callback',@(~,~) zoomX(2));
uicontrol('Style','pushbutton','String','Zoom In Y','Position',[290 row_y1 80 30],...
    'Callback',@(~,~) zoomY(0.5));
uicontrol('Style','pushbutton','String','Zoom Out Y','Position',[380 row_y1 80 30],...
    'Callback',@(~,~) zoomY(2));
uicontrol('Style','pushbutton','String','↑ Y','Position',[470 row_y1 40 30],...
    'Callback',@(~,~) shiftY(0.1));
uicontrol('Style','pushbutton','String','↓ Y','Position',[520 row_y1 40 30],...
    'Callback',@(~,~) shiftY(-0.1));

% Slider
uicontrol('Style','slider','Min',1,'Max',length(signal)-windowSize,...
    'Value',1,'SliderStep',[0.01 0.1],'Position',[580 row_y1 200 30],...
    'Callback',@(src,~) setStart(round(get(src,'Value'))));

% Threshold Controls
uicontrol('Style','text','String','Peak Thresh','Position',[800 row_y2 80 20]);
threshSlider = uicontrol('Style','slider','Min',0,'Max',10,'Value',peakThresh,...
    'Position',[800 row_y1 100 30],'Callback',@updateThresh);
threshEdit = uicontrol('Style','edit','String',num2str(peakThresh),...
    'Position',[910 row_y1 50 30],'Callback',@editThresh);

% Peak Window
uicontrol('Style','text','String','Window (ms)','Position',[970 row_y2 80 20]);
peakWinEdit = uicontrol('Style','edit','String','50','Position',[970 row_y1 50 30]);

% Threshold / Export Buttons
uicontrol('Style','pushbutton','String','Apply Threshold','Position',[1030 row_y2 120 30],...
    'Callback',@applyThreshold);
uicontrol('Style','pushbutton','String','Start Over','Position',[1160 row_y2 120 30],...
    'Callback',@startOver);
uicontrol('Style','pushbutton','String','Export & Close','Position',[1030 row_y1 120 30],...
    'Callback',@exportAndClose);

% Overlay Buttons
uicontrol('Style','pushbutton','String','Show Overlay','Position',[1290 row_y1 100 30],...
    'Callback',@(~,~) overlay_gui());
uicontrol('Style','pushbutton','String','Refresh Overlay','Position',[1400 row_y1 100 30],...
    'Callback',@(~,~) plotOverlay());

% === Refractory Time Text Field and Button ===
uicontrol(fig, 'Style', 'text', 'String', 'Refractory Time (ms):', ...
    'Position', [20 row_y2 140 20]);
refractoryField = uicontrol(fig, 'Style', 'edit', 'String', '0.1', ...
    'Position', [170 row_y2 60 25]);

uicontrol(fig, 'Style', 'pushbutton', 'String', 'Exclude Refractory', ...
    'Position', [240 row_y2 120 25], ...
    'Callback', @excludeRefractoryPeaks);

snapCheckbox = uicontrol(fig, 'Style', 'checkbox', 'String', 'Snap to local max', ...
    'Value', 1, ... % Checked by default
    'Position', [20 row_y2 150 25]);  % Adjust Y position as needed

% Plot Handles
% plotHandle = plot(ax, t, signal, 'k');
plotHandle = plotDownsampled(ax, t, signal, 100, 'k');
autoPeakPlot = plot(ax, nan, nan, 'ro');
manualPeakPlot = plot(ax, nan, nan, 'go');

overlayFig = [];  % handle to overlay figure
overlayAx = [];   % handle to overlay axes


applyThreshold();

%% Main GUI Functions
    function setStart(idx), startIdx = max(1, min(length(signal)-windowSize, idx)); updateView(); end
    function shiftX(factor), setStart(startIdx + round(factor * fs)); end
    function zoomX(factor), windowSize = max(round(fs * 0.1), min(length(signal), round(windowSize * factor))); updateView(); end
    function zoomY(factor), centerY = mean(yLimits); range = diff(yLimits) * factor / 2; yLimits = [centerY - range, centerY + range]; updateView(); end
    function shiftY(factor), shift = factor * diff(yLimits); yLimits = yLimits + shift; updateView(); end

    function updateThresh(~,~)
        peakThresh = get(threshSlider, 'Value');
        set(threshEdit, 'String', num2str(peakThresh));
        applyThreshold();
    end

function plotHandle = plotDownsampled(ax, x, y, stride, varargin)
    % Efficient line plotting for large data by downsampling
    if stride < 1, stride = 1; end
    x_ds = x(1:stride:end);
    y_ds = y(1:stride:end);
    plotHandle = plot(ax, x_ds, y_ds, varargin{:});
end

    function editThresh(~,~)
        val = str2double(get(threshEdit, 'String'));
        if ~isnan(val)
            peakThresh = min(10, max(0, val));
            set(threshSlider, 'Value', peakThresh);
            applyThreshold();
        end
    end

    function applyThreshold(~,~)
        idxRange = startIdx : min(startIdx+windowSize-1, length(signal));
        sigRange = signal(idxRange);
        autoPeaks = peakfinder(sigRange, peakThresh);
        autoTimes = t(idxRange(autoPeaks));
        autoValues = sigRange(autoPeaks);
        peakfinderValues = [autoTimes(:), autoValues(:)];
        updateView();
        plotOverlay();  % sync overlay
    end

    function startOver(~,~)
        manualPeaks = [];
        peakfinderValues = [];
        set(threshSlider, 'Value', 1);
        set(threshEdit, 'String', '1');
        updateView();
    end

function updateView(preserveXLim)
    if nargin < 1
        preserveXLim = false;
    end

    idxRange = startIdx : min(startIdx+windowSize-1, length(signal));
    tRange = t(idxRange);
    sigRange = signal(idxRange);
    set(plotHandle, 'XData', tRange, 'YData', sigRange);

    if ~isempty(peakfinderValues)
        autoTimes = peakfinderValues(:, 1);
        autoValues = peakfinderValues(:, 2);
        set(autoPeakPlot, 'XData', autoTimes, 'YData', autoValues);
    else
        set(autoPeakPlot, 'XData', NaN, 'YData', NaN);
    end

    if ~isempty(manualPeaks)
        visibleManual = manualPeaks(manualPeaks(:,1) >= tRange(1) & manualPeaks(:,1) <= tRange(end), :);
        set(manualPeakPlot, 'XData', visibleManual(:,1), 'YData', visibleManual(:,2));
    else
        set(manualPeakPlot, 'XData', NaN, 'YData', NaN);
    end

    if ~preserveXLim
        xlim(ax, [tRange(1), tRange(end)]);
    end

    ylim(ax, yLimits);
end

%     function updateView()
%         idxRange = startIdx : min(startIdx+windowSize-1, length(signal));
%         tRange = t(idxRange);
%         sigRange = signal(idxRange);
%         set(plotHandle, 'XData', tRange, 'YData', sigRange);
%         
%         if ~isempty(peakfinderValues)
%             autoTimes = peakfinderValues(:, 1);
%             autoValues = peakfinderValues(:, 2);
%             set(autoPeakPlot, 'XData', autoTimes, 'YData', autoValues);
%         else
%             set(autoPeakPlot, 'XData', NaN, 'YData', NaN);
%         end
%         
%         if ~isempty(manualPeaks)
%             visibleManual = manualPeaks(manualPeaks(:,1) >= tRange(1) & manualPeaks(:,1) <= tRange(end), :);
%             set(manualPeakPlot, 'XData', visibleManual(:,1), 'YData', visibleManual(:,2));
%         else
%             set(manualPeakPlot, 'XData', NaN, 'YData', NaN);
%         end
%         
%         xlim(ax, [tRange(1), tRange(end)]);
%         ylim(ax, yLimits);
%     end

%     function plotOverlay()
%         if isempty(overlayFig) || ~isvalid(overlayFig)
%             return;
%         end
%         
%         cla(overlayAx);
%         hold(overlayAx, 'on');
%         for i = 1:size(peakfinderValues,1)
%             pk = round(peakfinderValues(i,1) * fs);
%             idx = (pk-halfWin):(pk+halfWin);
%             if min(idx)<1 || max(idx)>length(signal), continue; end
%             seg = signal(idx) - signal(pk);
%             plot(overlayAx, tWin, seg, 'r');
%         end
%         for i = 1:size(manualPeaks,1)
%             pk = round(manualPeaks(i,1) * fs);
%             idx = (pk-halfWin):(pk+halfWin);
%             if min(idx)<1 || max(idx)>length(signal), continue; end
%             seg = signal(idx) - signal(pk);
%             plot(overlayAx, tWin, seg, 'g');
%         end
%     end


function plotOverlay()
    stride = strideValues.overlay;   % in plotOverlay
    
    if isempty(overlayFig) || ~isvalid(overlayFig)
        return;
    end

    cla(overlayAx);
    hold(overlayAx, 'on');

    stride = 5;
    tDown = tWin(1:stride:end);

    % Preallocate
    numAuto = size(peakfinderValues,1);
    numManual = size(manualPeaks,1);
    allLines = gobjects(numAuto + numManual, 1);

    % Plot auto-detected peaks
    for i = 1:numAuto
        pk = round(peakfinderValues(i,1) * fs);
        idx = (pk-halfWin):(pk+halfWin);
        if min(idx)<1 || max(idx)>length(signal), continue; end
        seg = signal(idx) - signal(pk);
        segDown = seg(1:stride:end);
        allLines(i) = line(overlayAx, tDown, segDown, 'Color', 'r');
    end

    % Plot manual peaks
    for i = 1:numManual
        pk = round(manualPeaks(i,1) * fs);
        idx = (pk-halfWin):(pk+halfWin);
        if min(idx)<1 || max(idx)>length(signal), continue; end
        seg = signal(idx) - signal(pk);
        segDown = seg(1:stride:end);
        allLines(numAuto + i) = line(overlayAx, tDown, segDown, 'Color', 'g');
    end
end

function updateStride(src, target)
    val = str2double(get(src, 'String'));
    if isnan(val) || val < 1
        warndlg('Stride must be a positive integer.', 'Invalid Input');
        set(src, 'String', '3');
        val = 3;
    end
    val = round(val);
    strideValues.(target) = val;

    % Optionally trigger a redraw:
    switch target
        case 'overlay'
            plotOverlay();  % this should use strideValues.overlay internally
        case 'analysis'
            updateAnalysisPlot(currentSimThresh, currentMethod); % also uses stride
    end
end

function updatelowTresh(src, target)
    val = str2double(get(src, 'String'));
    if isnan(val) || val <= 0
    warndlg('lowThresh must be a positive number.', 'Invalid Input');
    set(src, 'String', num2str(lowThresh.(target)));
    return;
end
lowThresh.(target) = val;

    % Optionally trigger a redraw:
    switch target
        case 'overlay'
            plotOverlay();  % this should use strideValues.overlay internally
        case 'analysis'
            updateAnalysisPlot(currentSimThresh, currentMethod); % also uses stride
    end
end


    function mouseClick(~,~)
    coords = get(ax, 'CurrentPoint');
    xClick = coords(1,1); yClick = coords(1,2);
    clickType = get(fig, 'SelectionType');

    % Clear previous dashed range markers if they exist
    delete(findall(ax, 'Tag', 'clickRangeLine'));

    switch clickType
        case 'normal'
            winMs = str2double(get(peakWinEdit, 'String'));
            winSamps = round((winMs/1000) * fs);
            centerIdx = round(xClick * fs);
            fromIdx = max(1, centerIdx - winSamps);
            toIdx = min(length(signal), centerIdx + winSamps);

            % Plot range as dashed lines
            x1 = t(fromIdx);
            x2 = t(toIdx);
            yLims = ylim(ax);
            line(ax, [x1 x1], yLims, 'Color', [0.4 0.4 0.4], 'LineStyle', '--', 'Tag', 'clickRangeLine');
            line(ax, [x2 x2], yLims, 'Color', [0.4 0.4 0.4], 'LineStyle', '--', 'Tag', 'clickRangeLine');

            if get(snapCheckbox, 'Value')  % Snap enabled
                [~, localPeak] = max(signal(fromIdx:toIdx));
                peakIdx = fromIdx + localPeak - 1;
                manualPeaks(end+1,:) = [t(peakIdx), signal(peakIdx)];
            else  % No snapping, take clicked position
                manualPeaks(end+1,:) = [xClick, yClick];
            end

            updateView();
            plotOverlay();

        case 'alt'
            allPeaks = [peakfinderValues; manualPeaks];
            if isempty(allPeaks), return; end

            distances = abs(allPeaks(:,1) - xClick);
%             distances = vecnorm(allPeaks - [xClick, yClick], 2, 2);
            [~, closestIdx] = min(distances);  % <-- moved here

            if distances(closestIdx) < (str2double(get(peakWinEdit, 'String')))/100
                if closestIdx <= size(peakfinderValues, 1)
                    peakfinderValues(closestIdx, :) = [];
                else
                    manualPeaks(closestIdx - size(peakfinderValues, 1), :) = [];
                end
            end
            updateView();
            plotOverlay();
%     case 'extend'
%     % Middle-click to highlight nearest peak in x-direction
%     allPeaks = [peakfinderValues; manualPeaks];
%     if isempty(allPeaks), return; end
% 
%     % Find closest peak by x
%     distances = abs(allPeaks(:,1) - xClick);
%     [~, closestIdx] = min(distances);
%     selectedTime = allPeaks(closestIdx,1);
%     pk = round(selectedTime * fs);
%     if pk < 1 || pk > length(signal), return; end
% 
%     % Remove old highlights
%     delete(findall(ax, 'Tag', 'tempHighlight'));
%     delete(findall(overlayAx, 'Tag', 'tempHighlight'));
% 
%     % Highlight in main figure
%     yLims = ylim(ax);
%     line(ax, [selectedTime selectedTime], yLims, ...
%         'Color', [0 0 1], 'LineStyle', '--', 'LineWidth', 1.5, 'Tag', 'tempHighlight');
%     hold(ax, 'on');
%     plot(ax, selectedTime, signal(pk), 'o', 'Color', [0 0 1], 'MarkerSize', 6, 'Tag', 'tempHighlight');

    % Center view
%     winMs = str2double(get(peakWinEdit, 'String'));
%     halfWinSec = 5;%(winMs / 1000) / 2;
%     xlim(ax, [selectedTime - halfWinSec, selectedTime + halfWinSec]);
% currXLim = xlim(ax);
% winWidth = diff(currXLim) / 2;
% xlim(ax, [selectedTime - winWidth, selectedTime + winWidth]);
% updateView(true);
% Center the main view around selectedTime

case 'extend'
    % Middle-click to highlight nearest peak in x-direction
    allPeaks = [peakfinderValues; manualPeaks];
    if isempty(allPeaks), return; end

    % Find closest peak by x
    distances = abs(allPeaks(:,1) - xClick);
    [~, closestIdx] = min(distances);
    selectedTime = allPeaks(closestIdx,1);
    pk = round(selectedTime * fs);
    if pk < 1 || pk > length(signal), return; end

    % Remove old highlights
    delete(findall(ax, 'Tag', 'tempHighlight'));
    delete(findall(overlayAx, 'Tag', 'tempHighlight'));

    % Highlight in main figure
    yLims = ylim(ax);
    line(ax, [selectedTime selectedTime], yLims, ...
        'Color', [0 0 1], 'LineStyle', '--', 'LineWidth', 1.5, 'Tag', 'tempHighlight');
    hold(ax, 'on');
    plot(ax, selectedTime, signal(pk), 'o', 'Color', [0 0 1], 'MarkerSize', 6, 'Tag', 'tempHighlight');

    % Smooth center on peak
    currentX = xlim(ax);
    targetCenter = selectedTime;
    halfWidth = diff(currentX) / 2;
    targetX = [targetCenter - halfWidth, targetCenter + halfWidth];

    steps = 20;
    for i = 1:steps
        newX = currentX + (targetX - currentX) * (i / steps);
        xlim(ax, newX);
        drawnow;
        pause(0.01);  % Adjust for speed/smoothness
    end

    % Replot signal within new view
    startIdx = max(1, round((targetX(1)) * fs));
    updateView(false);

    % Plot corresponding snippet in overlay
    idx = (pk-halfWin):(pk+halfWin);
    if min(idx)<1 || max(idx)>length(signal), return; end
    seg = signal(idx) - signal(pk);
    stride = strideValues.overlay;
    tDown = tWin(1:stride:end);
    segDown = seg(1:stride:end);
    plot(overlayAx, tDown, segDown, 'Color', [0 0 1], 'LineWidth', 2, 'Tag', 'tempHighlight');
    
currWinWidth = diff(xlim(ax));
newCenterTime = selectedTime;
startIdx = max(1, round((newCenterTime - currWinWidth/2) * fs));
startIdx = min(startIdx, length(signal) - windowSize + 1);  % stay within bounds

updateView(false);  % now it will replot the correct portion of the signal

    % Plot corresponding snippet in overlay
    idx = (pk-halfWin):(pk+halfWin);
    if min(idx)<1 || max(idx)>length(signal), return; end
    seg = signal(idx) - signal(pk);
    stride = strideValues.overlay;
    tDown = tWin(1:stride:end);
    segDown = seg(1:stride:end);
    plot(overlayAx, tDown, segDown, 'Color', [0 0 1], 'LineWidth', 2, 'Tag', 'tempHighlight');

    end
end

    function exportAndClose(~,~)
        idxRange = startIdx : min(startIdx+windowSize-1, length(signal));
        tRange = t(idxRange);
        sigRange = signal(idxRange);
        autoPeaks = peakfinder(sigRange, peakThresh);
        autoTimes = tRange(autoPeaks);
        if isempty(manualPeaks), manualPeaks(1,1) = NaN; end
%         allTimes = unique([autoTimes(:); manualPeaks(:,1); peakfinderValues(:,1)]);
        allTimes = unique([manualPeaks(:,1); peakfinderValues(:,1)]);
        assignin('base', 'AllPeakTimes', allTimes);
        assignin('base', 'ManualPeakTimes', manualPeaks(:,1));
        assignin('base', 'PeakfinderPeakTimes', peakfinderValues(:,1));
        assignin('base', 'PeakSensitivity', peakThresh);
        close(fig);
        close(overlayFig);
    end

    function excludeRefractoryPeaks(~,~)
        % --- Get refractory time from field ---
        refractoryTimeMs = str2double(get(refractoryField, 'String'));
        if isnan(refractoryTimeMs) || refractoryTimeMs <= 0
            errordlg('Please enter a valid refractory time in milliseconds.','Invalid Input');
            return;
        end
        refractorySamples = round(refractoryTimeMs * fs / 1000);  % convert ms to samples
        
        % === Sort and clean manualPeaks ===
        if ~isempty(manualPeaks)
            manualPeaks = sortrows(manualPeaks, 1);  % by time
            manualPeaks = removeClosePeaks(manualPeaks, refractorySamples);
        end
        
        % === Sort and clean peakfinderValues ===
        if ~isempty(peakfinderValues)
            peakfinderValues = sortrows(peakfinderValues, 1);
            peakfinderValues = removeClosePeaks(peakfinderValues, refractorySamples);
        end
        
        % === Remove peakfinderValues too close to manualPeaks ===
        if ~isempty(manualPeaks) && ~isempty(peakfinderValues)
            keepMask = true(size(peakfinderValues, 1), 1);
            for i = 1:size(peakfinderValues, 1)
                pfPeakTime = peakfinderValues(i, 1);
                timeDiffs = abs(manualPeaks(:, 1) - pfPeakTime) * fs;
                if any(timeDiffs < refractorySamples)
                    keepMask(i) = false;
                end
            end
            peakfinderValues = peakfinderValues(keepMask, :);
        end
        
        % --- Update the plots ---
        updateView();
        plotOverlay();
    end


    function cleanedPeaks = removeClosePeaks(peaks, refractorySamples)
        if isempty(peaks)
            cleanedPeaks = peaks;
            return;
        end
        times = peaks(:,1) * fs;  % convert to samples
        keep = true(size(times));
        lastKept = times(1);
        
        for i = 2:length(times)
            if times(i) - lastKept < refractorySamples
                keep(i) = false;  % too close, remove
            else
                lastKept = times(i);
            end
        end
        cleanedPeaks = peaks(keep, :);
    end


%% NESTED FUNCTION: Overlay GUI
    function overlay_gui()
        if isempty(overlayFig) || ~isvalid(overlayFig)
            overlayFig = figure('Name','Peak Overlay','NumberTitle','off',...
                'WindowButtonDownFcn',@mouseClickOverlay);
            overlayAx = axes('Parent', overlayFig, ...
            'Units', 'normalized', ...
            'Position', [0.05, 0.25, 0.9, 0.7]);  % [left, bottom, width, height]
        
            hold(overlayAx, 'on');
            
            % Shift the axes up to make space for UI buttons
            axPos = get(overlayAx, 'Position');  % [left bottom width height]
            axPos(2) = axPos(2) + 0.15;  % shift bottom up
            axPos(4) = axPos(4) - 0.15;  % reduce height to maintain top edge
            set(overlayAx, 'Position', axPos);
            
            
            xlabel(overlayAx, 'Time (ms)'); ylabel(overlayAx, 'Amplitude');
            title(overlayAx, 'Aligned Peaks');
        else
            figure(overlayFig); % bring to front
        end
        
        % UI Button Base Settings
btnWidth = 80;
btnHeight = 30;
btnSpacingX = 10;
btnSpacingY = 10;
startX = 50;
rowY1 = 10;
rowY2 = rowY1 + btnHeight + btnSpacingY;

% Row 1
uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Zoom In X', ...
    'Position', [startX rowY1 btnWidth btnHeight], ...
    'Callback', @(~,~) zoomOverlayX(0.5));

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Zoom Out X', ...
    'Position', [startX + (btnWidth+btnSpacingX)*1 rowY1 btnWidth btnHeight], ...
    'Callback', @(~,~) zoomOverlayX(2));

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Zoom In Y', ...
    'Position', [startX + (btnWidth+btnSpacingX)*2 rowY1 btnWidth btnHeight], ...
    'Callback', @(~,~) zoomOverlayY(0.5));

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Zoom Out Y', ...
    'Position', [startX + (btnWidth+btnSpacingX)*3 rowY1 btnWidth btnHeight], ...
    'Callback', @(~,~) zoomOverlayY(2));

% Row 2
uicontrol(overlayFig, 'Style', 'pushbutton', 'String', '↑ Y', ...
    'Position', [startX rowY2 40 btnHeight], ...
    'Callback', @(~,~) shiftOverlayY(0.1));

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', '↓ Y', ...
    'Position', [startX + 50 rowY2 40 btnHeight], ...
    'Callback', @(~,~) shiftOverlayY(-0.1));

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Fit Data', ...
    'Position', [startX + (btnWidth+btnSpacingX)*2 rowY2 btnWidth btnHeight], ...
    'Callback', @(~,~) fitAndAnalyzePeaks());

uicontrol(overlayFig, 'Style', 'pushbutton', 'String', 'Exclude Line', ...
    'Position', [startX + (btnWidth+btnSpacingX)*3 rowY2 btnWidth btnHeight], ...
    'Callback', @startExcludeLineMode);

% Default lowTresh Value for Peak Detection in Fit
lowThreshEdit = uicontrol(overlayFig, 'Style', 'edit', ...
    'String', num2str(lowThresh.overlay), ...
    'Position', [startX + (btnWidth+btnSpacingX)*4 rowY1 btnWidth btnHeight], ...
    'TooltipString', 'lowThresh for Spike Detection in Fitting Analysis', ...
    'Callback', @(src,~) updatelowTresh(src, 'overlay'));


% Default stride value
% defaultStride = 3;
strideOverlayEdit = uicontrol(overlayFig, 'Style', 'edit', 'String', num2str(defaultStride), ...
    'Position', [startX + (btnWidth+btnSpacingX)*4 rowY2 btnWidth btnHeight], 'TooltipString', 'Downsampling stride', ...
    'Callback', @(src,~) updateStride(src, 'overlay'));
        
        
        plotOverlay();
        
        function startExcludeLineMode(~,~)
            % Instruct the user
            disp('Draw a line across the plot to exclude intersecting peaks...');
            
            % Let the user draw a line
            hLine = drawline('Color', 'r', 'LineWidth', 1.5);
            
            % Get line coordinates
            linePos = hLine.Position; % [x1 y1; x2 y2]
            
            % Get line equation (ax + by + c = 0)
            dx = linePos(2,1) - linePos(1,1);
            dy = linePos(2,2) - linePos(1,2);
            if dx == 0 && dy == 0, return; end  % avoid degenerate line
            
            
            % Loop through peaks in manualPeaks and peakfinderValues
            manualPeaks = excludePeaksOnLine(manualPeaks, 'manual');
            peakfinderValues = excludePeaksOnLine(peakfinderValues, 'detected');
            
            % Delete line after use
            delete(hLine);
            
            % Replot everything
            updateView();
            plotOverlay();
            
            function updatedPeaks = excludePeaksOnLine(peaks_found, label)
                updatedPeaks = peaks_found;
                if isempty(peaks_found), return; end
                
                % Line coordinates from overlay
                x1 = linePos(1,1); y1 = linePos(1,2);
                x2 = linePos(2,1); y2 = linePos(2,2);
                
                keepMask = true(size(peaks_found, 1),1);
                
                
                for i = 1:size(peaks_found,1)
                    % First recalculate the x and y oordinates from the overlay
                                pk = round(peaks_found(i,1) * fs);
                                idx = (pk-halfWin):(pk+halfWin);
                                if min(idx)<1 || max(idx)>length(signal), continue; end
                                seg = signal(idx) - signal(pk);
                                segTime = tWin;
                          
                    
%                     % Convert peak time to index
%                     peakIdx = round(peaks_found(i,1) * fs);
%                     idx = (peakIdx - halfWin):(peakIdx + halfWin);
%                     if min(idx)<1 || max(idx)>length(signal)
%                         continue;
%                     end
%                     seg = signal(idx);
%                     segTime = t(idx);
                    
                    % Loop through each pair of adjacent points in the segment
                    for j = 1:length(segTime)-1
                        xA = segTime(j); yA = seg(j);
                        xB = segTime(j+1); yB = seg(j+1);
                        
                        if segmentsIntersect(xA, yA, xB, yB, x1, y1, x2, y2)
                            keepMask(i) = false;
                            break; % no need to check further
                        end
                    end
                end
                
                updatedPeaks = peaks_found(keepMask,:);
            end
            
            % This function checks if two line segments (x1, y1) -> (x2, y2) and (x3, y3) -> (x4, y4) intersect
            function intersect = segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)
                % Compute direction of each segment
                d1 = direction(x3, y3, x4, y4, x1, y1);
                d2 = direction(x3, y3, x4, y4, x2, y2);
                d3 = direction(x1, y1, x2, y2, x3, y3);
                d4 = direction(x1, y1, x2, y2, x4, y4);
                
                % If the segments are intersecting, the directions must be opposite in sign
                intersect = ((d1 * d2 < 0) && (d3 * d4 < 0));
            end
            
            % Helper function to compute the direction of the point (x, y) with respect to the line segment (x1, y1) -> (x2, y2)
            function d = direction(x1, y1, x2, y2, x3, y3)
                d = (x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1);
            end
            function result = isPointNearLine(px, py, x1, y1, x2, y2)
                % Minimum distance from point to line segment
                A = [x2 - x1, y2 - y1];
                B = [px - x1, py - y1];
                t = max(0, min(1, dot(A,B) / dot(A,A)));
                closestPoint = [x1 y1] + t * A;
                dist = norm([px py] - closestPoint);
                result = dist < 0.01; % Adjust threshold based on scale
            end
        end
    end

    function mouseClickOverlay(~,~)
    coords = get(overlayAx, 'CurrentPoint');
    xClick = coords(1,1); yClick = coords(1,2);
    clickType = get(overlayFig, 'SelectionType');
    
    if strcmp(clickType, 'alt')  % Right-click to delete peak
        % [existing delete logic unchanged...]
        % ...
        updateView();
        plotOverlay();
        return;
    end

    if ~strcmp(clickType, 'extend')  % Only handle middle click
        return;
    end

    % Remove old highlight
    delete(findall(overlayAx, 'Tag', 'tempHighlight'));
    delete(findall(ax, 'Tag', 'tempHighlight'));

    % Find closest peak in (x,y)
    allPeaks = [peakfinderValues; manualPeaks];
    if isempty(allPeaks), return; end

    dists = [];
    for i = 1:size(allPeaks,1)
        pk = round(allPeaks(i,1) * fs);
        idx = (pk-halfWin):(pk+halfWin);
        if min(idx)<1 || max(idx)>length(signal)
            dists(end+1) = inf; continue;
        end
        seg = signal(idx) - signal(pk);
        [~, ti] = min(abs(tWin - xClick));
        d = hypot(tWin(ti)-xClick, seg(ti)-yClick);
        dists(end+1) = d;
    end

    [~, closestIdx] = min(dists);
    selectedTime = allPeaks(closestIdx,1);
    pk = round(selectedTime * fs);
    idx = (pk-halfWin):(pk+halfWin);
    if min(idx)<1 || max(idx)>length(signal), return; end

    % Plot temp highlight in overlay
    seg = signal(idx) - signal(pk);
    stride = strideValues.overlay;
    tDown = tWin(1:stride:end);
    segDown = seg(1:stride:end);
    plot(overlayAx, tDown, segDown, 'Color', [0 0 1], 'LineWidth', 2, 'Tag', 'tempHighlight');

    % Plot vertical dashed line and highlight point in main figure
    yLims = ylim(ax);
    line(ax, [selectedTime selectedTime], yLims, ...
        'Color', [0 0 1], 'LineStyle', '--', 'LineWidth', 1.5, 'Tag', 'tempHighlight');
    hold(ax, 'on');
    plot(ax, selectedTime, signal(pk), 'o', 'Color', [0 0 1], 'MarkerSize', 6, 'Tag', 'tempHighlight');

    % Smooth pan view to center on selected peak
currX = xlim(ax);
currWidth = diff(currX);
targetCenter = selectedTime;
targetX = [targetCenter - currWidth/2, targetCenter + currWidth/2];

steps = 20;
for i = 1:steps
    newX = currX + (targetX - currX) * (i / steps);
    xlim(ax, newX);
    drawnow;
    pause(0.01);  % Adjust speed here
end

% Update signal display without resetting x-limits
startIdx = max(1, round(targetX(1) * fs));
updateView(false);

%     % Center view on selected peak
% currWinWidth = diff(xlim(ax));
% newCenterTime = selectedTime;
% startIdx = max(1, round((newCenterTime - currWinWidth/2) * fs));
% startIdx = min(startIdx, length(signal) - windowSize + 1);  % stay within bounds
% 
% updateView(false);  % now it will replot the correct portion of the signal
end


    function zoomOverlayX(factor)
        xl = xlim(overlayAx);
        centerX = mean(xl);
        rangeX = diff(xl) * factor / 2;
        xlim(overlayAx, [centerX - rangeX, centerX + rangeX]);
    end

    function zoomOverlayY(factor)
        yl = ylim(overlayAx);
        centerY = mean(yl);
        rangeY = diff(yl) * factor / 2;
        ylim(overlayAx, [centerY - rangeY, centerY + rangeY]);
    end

    function shiftOverlayY(factor)
        yl = ylim(overlayAx);
        shift = factor * diff(yl);
        ylim(overlayAx, yl + shift);
    end

    function fitAndAnalyzePeaks()
        % === Get All Verified Peaks from overlay ===
        verifiedPeaks = [peakfinderValues; manualPeaks];
        verifiedAligned = [];
        orangeSegHandles = struct('pk', {}, 'handle', {}, 'isSimilar', {});
        
        for i = 1:size(verifiedPeaks,1)
            pk = round(verifiedPeaks(i,1) * fs);
            idx = (pk - halfWin):(pk + halfWin);
            if min(idx)<1 || max(idx)>length(signal), continue; end
            seg = signal(idx) - signal(pk);
            verifiedAligned(end+1,:) = seg;
        end
        
        % === Compute Mean and Standard Error from Verified Peaks ===
        meanPeakShape = mean(verifiedAligned, 1);
        stePeakShape = std(verifiedAligned, 0, 1) / sqrt(size(verifiedAligned, 1));
        fillArea = [meanPeakShape + stePeakShape, fliplr(meanPeakShape - stePeakShape)];
        fillX = [tWin, fliplr(tWin)];
        
        % === Run New Low-Threshold Peakfinder ===
%         lowThresh = 0.5;
        orangePeaks = peakfinder(signal, lowThresh.overlay, [], 1, false); % returns sample indices
        
        % === Setup Analysis Figure ===
        analysisFig = figure('Name','Peak Shape Analysis','NumberTitle','off',...
            'CloseRequestFcn', @closeAnalysis);
        analysisAx = axes('Parent', analysisFig, 'Units', 'normalized', ...
    'Position', [0.05, 0.25, 0.9, 0.7]); hold(analysisAx, 'on');
        xlabel(analysisAx, 'Time (ms)'); ylabel(analysisAx, 'Amplitude');
        title(analysisAx, 'Similarity to Mean Peak');
        
        % === UI Elements ===
        currentMethod = 'Pearson Correlation';
        uicontrol(analysisFig, 'Style', 'text', 'String', 'Similarity Method', ...
            'Position', [20 50 100 20]);
        methodDropdown = uicontrol(analysisFig, 'Style', 'popupmenu', ...
            'String', {'Pearson Correlation', 'Euclidean Distance', 'Cosine Similarity'}, ...
            'Position', [150 50 200 25], ...
            'Callback', @(src,~) onMethodChange(src.String{src.Value}));
        uicontrol(analysisFig, 'Style', 'text', 'String', 'Similarity Threshold', ...
            'Position', [20 20 120 20]);
        similaritySlider = uicontrol(analysisFig, 'Style', 'slider', ...
            'Min', 0.5, 'Max', 1, 'Value', 0.9, ...
            'Position', [150 20 200 20], ...
            'Callback', @(src,~) updateAnalysisPlot(src.Value, currentMethod));
        uicontrol(analysisFig, 'Style', 'pushbutton', 'String', 'Add', ...
            'Position', [370 50 100 25], ...
            'Callback', @addSelectedPeaks);
        
%         defaultStride = 3;
        strideAnalysisEdit = uicontrol(analysisFig, 'Style', 'edit', 'String', num2str(defaultStride), ...
    'Position', [370 20 100 25], 'TooltipString', 'Downsampling stride', ...
    'Callback', @(src,~) updateStride(src, 'analysis'));
        
        % Mouse callback
        set(analysisFig, 'WindowButtonDownFcn', @onMouseClick);
        
        % Initial plot
        updateAnalysisPlot(similaritySlider.Value, currentMethod);
        
        function onMethodChange(newMethod)
            currentMethod = newMethod;
            updateAnalysisPlot(similaritySlider.Value, currentMethod);
        end
        
%         function updateAnalysisPlot(simThresh, method)
%             cla(analysisAx); hold(analysisAx, 'on');
%             orangePeaksFiltered = [];
%             
%             % Clear previous handles
%             orangeSegHandles = struct('pk', {}, 'handle', {}, 'isSimilar', {});
%             
%             % === Plot fill + mean shape first ===
%             fill(analysisAx, fillX, fillArea, [0.6 0.6 0.6], ...
%                 'EdgeColor', 'none', 'FaceAlpha', 0.3);
%             plot(analysisAx, tWin, meanPeakShape, 'k', 'LineWidth', 2);
%             
%             % === Plot magenta or orange based on similarity ===
%             for i = 1:numel(orangePeaks)
%                 pk = round(orangePeaks(i));
%                 idx = (pk - halfWin):(pk + halfWin);
%                 if min(idx)<1 || max(idx)>length(signal), continue; end
%                 seg = signal(idx) - signal(pk);
%                 
%                 switch method
%                     case 'Pearson Correlation'
%                         r = corr(seg(:), meanPeakShape(:));
%                         similarity = r;
%                     case 'Euclidean Distance'
%                         d = norm(seg - meanPeakShape);
%                         similarity = 1 / (1 + d); % inverse distance
%                     case 'Cosine Similarity'
%                         similarity = dot(seg, meanPeakShape) / ...
%                             (norm(seg) * norm(meanPeakShape));
%                 end
%                 
%                 isSimilar = similarity >= simThresh;
%                 color = isSimilar * [1 0.5 0] + ~isSimilar * [1 0 1];  % orange or magenta
%                 h = plot(analysisAx, tWin, seg, 'Color', color);
%                 orangeSegHandles(end+1) = struct('pk', pk, 'handle', h, 'isSimilar', isSimilar);
%             end
%         end

function updateAnalysisPlot(simThresh, method)
    cla(analysisAx); 
    hold(analysisAx, 'on');
    
    lightOrange = [1 0.75 0.3];
    lightMagenta = [1 0.5 1];
    
    stride = strideValues.analysis; % in updateAnalysisPlot
    
    % Clear previous handles
    orangeSegHandles = struct('pk', {}, 'handle', {}, 'isSimilar', {});


    % === Preallocate for speed ===
    stride = 5;  % downsampling stride
    tDown = tWin(1:stride:end);
    segLength = length(tDown);
    validPeaks = false(1, numel(orangePeaks));
    segMatrix = zeros(numel(orangePeaks), segLength);
    similarityScores = zeros(1, numel(orangePeaks));

    % === Gather segments and compute similarities ===
    for i = 1:numel(orangePeaks)
        pk = round(orangePeaks(i));
        idx = (pk - halfWin):(pk + halfWin);
        if min(idx)<1 || max(idx)>length(signal), continue; end
        seg = signal(idx) - signal(pk);
        segDown = seg(1:stride:end);

        % Similarity computation
        switch method
            case 'Pearson Correlation'
                r = corr(seg(:), meanPeakShape(:));
                similarity = r;
            case 'Euclidean Distance'
                d = norm(seg - meanPeakShape);
                similarity = 1 / (1 + d);
            case 'Cosine Similarity'
                similarity = dot(seg, meanPeakShape) / ...
                             (norm(seg) * norm(meanPeakShape));
        end

        validPeaks(i) = true;
        similarityScores(i) = similarity;
        segMatrix(i, :) = segDown;
    end

    % === Plot all similar (orange) and dissimilar (magenta) ===
    isSimilar = similarityScores >= simThresh;
    segSimilar = segMatrix(isSimilar & validPeaks, :);
    segOther = segMatrix(~isSimilar & validPeaks, :);
    
    % === Plot fill + mean shape  ===
    fill(analysisAx, fillX, fillArea, [0.6 0.6 0.6], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.3);
    plot(analysisAx, tWin, meanPeakShape, 'k', 'LineWidth', 2);

    % Plot with color grouping
    if ~isempty(segSimilar)
%         plot(analysisAx, tDown, segSimilar', 'Color', [1 0.5 0]); % orange
plot(analysisAx, tDown, segSimilar', 'Color', lightOrange, 'LineWidth', 0.1);
    end
    if ~isempty(segOther)
%         plot(analysisAx, tDown, segOther', 'Color', [1 0 1]); % magenta
        plot(analysisAx, tDown, segOther', 'Color', lightMagenta, 'LineWidth', 0.1);
    end

    % Store handles if needed (optional depending on use)
    for i = find(validPeaks)
        orangeSegHandles(end+1) = struct( ...
            'pk', round(orangePeaks(i)), ...
            'handle', [], ...
            'isSimilar', isSimilar(i));
    end
end

        
        function onMouseClick(~,~)
            coords = get(analysisAx, 'CurrentPoint');
            xClick = coords(1,1); yClick = coords(1,2);
            dists = arrayfun(@(h) min(hypot(tWin - xClick, get(h.handle, 'YData') - yClick)), orangeSegHandles);
            [~, idx] = min(dists);
            % Toggle visibility or mark status
            if orangeSegHandles(idx).isSimilar
                % Add this peak to manualPeaks
                peakTime = orangeSegHandles(idx).pk / fs;
                manualPeaks(end+1,:) = [peakTime, signal(orangeSegHandles(idx).pk)];
                updateView();
                plotOverlay();
            end
        end
        
        function addSelectedPeaks(~,~)
            for i = 1:numel(orangeSegHandles)
                if orangeSegHandles(i).isSimilar
                    peakTime = orangeSegHandles(i).pk / fs;
                    manualPeaks(end+1,:) = [peakTime, signal(orangeSegHandles(i).pk)];
                end
            end
            tic
            updateView();
            toc
            tic
            plotOverlay();
            toc
            close(analysisFig); % <-- Add this line to close the figure
        end
        
        function closeAnalysis(analysisFig,~)
            if ishandle(analysisFig)
                delete(analysisFig);
            end
        end
    end




plotOverlay();
end

