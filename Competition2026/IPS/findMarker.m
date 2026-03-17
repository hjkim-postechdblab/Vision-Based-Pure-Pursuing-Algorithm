function [ex, ey, Flag_marker] = findMarker(IMG_marker_eroded, ...
    COG_X, COG_Y, MARKER_MIN_PIXELS)
%findMarker  Detect landing marker after disk erosion
%   IMG_marker_eroded: logical H x W (disk-eroded binary frame)
%   ex, ey: marker error from frame center [pixel]
%   Flag_marker: boolean - marker detected
%
% Reference: tech_spec.md Section 3.5
% MARKER_MIN_PIXELS=10 prevents noise blob false positives (Deviation D2)
% Parameters: variables.m (MARKER_MIN_PIXELS=10, COG_X=60, COG_Y=80)

    [mark_rows, mark_cols] = find(IMG_marker_eroded);

    if numel(mark_rows) >= MARKER_MIN_PIXELS
        x_MARK = mean(mark_rows);
        y_MARK = mean(mark_cols);
        ex = x_MARK - COG_X;
        ey = y_MARK - COG_Y;
        Flag_marker = true;
    else
        ex = 0;
        ey = 0;
        Flag_marker = false;
    end
end
