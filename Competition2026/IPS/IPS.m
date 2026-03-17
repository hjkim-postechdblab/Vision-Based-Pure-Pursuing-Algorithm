function [ex, ey, Flag_VTP, Flag_marker] = IPS(IMG)
%IPS  Image Processing System - camera frame to error/flag outputs
%   IMG: uint8 H x W x 3 RGB frame (T_IPS = 0.2s, 5Hz)
%   ex, ey: double - row/col direction error [pixel]
%   Flag_VTP: boolean - track (VTP) detected
%   Flag_marker: boolean - landing marker detected
%
% Reference: tech_spec.md Section 3.6
% Internal state: theta_prev (persistent, initialized to NaN)
%
% Pipeline: Channel Convert -> Binarize -> Erosion -> VTP Search -> Marker Detection
% Flag mutual exclusivity: Flag_VTP and Flag_marker cannot both be true

    persistent theta_prev
    if isempty(theta_prev)
        theta_prev = NaN;  % Initial: 360 degree full search mode
    end

    % Step 1+2: Channel conversion + Binarization
    Fbin = channelConvertAndBinarize(IMG, G_B_GAIN, BINARIZER_THRESHOLD);

    % Step 3: Erosion (square for track, disk for marker)
    [IMG_eroded, IMG_marker_eroded] = erodeTrackAndMarker(Fbin, ...
        SQUARE_KERNEL, DISK_KERNEL);

    % Step 4: VTP search (Arc Mask)
    [ex_vtp, ey_vtp, Flag_VTP, theta_next] = findVTP(IMG_eroded, ...
        theta_prev, MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, ...
        COG_X, COG_Y, FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);

    % Step 5: Marker detection
    [ex_mark, ey_mark, Flag_marker] = findMarker(IMG_marker_eroded, ...
        COG_X, COG_Y, MARKER_MIN_PIXELS);

    % Flag mutual exclusivity: VTP takes priority
    if Flag_VTP
        ex = ex_vtp;
        ey = ey_vtp;
        Flag_marker = false;       % Suppress marker when VTP exists
        theta_prev = theta_next;   % Update direction
    elseif Flag_marker
        ex = ex_mark;
        ey = ey_mark;
        Flag_VTP = false;          % Explicit false
        % theta_prev not updated (direction irrelevant during marker tracking)
    else
        ex = 0;
        ey = 0;
        % theta_prev preserved (keep last known direction on track loss)
    end
end
