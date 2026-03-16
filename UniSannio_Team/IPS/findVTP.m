function [ex, ey, Flag_VTP, theta_next] = findVTP(IMG_eroded, theta_prev, ...
    MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y, ...
    FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH)
%findVTP  Arc Mask based Virtual Target Point search
%   IMG_eroded: logical H x W (square-eroded binary frame)
%   theta_prev: double - previous VTP direction [rad], NaN = 360 deg search
%   ex, ey: VTP error from frame center [pixel]
%   Flag_VTP: boolean - VTP detected
%   theta_next: double - updated VTP direction [rad]
%
% Reference: tech_spec.md Section 3.4.1~3.4.5
% Parameters: variables.m (FOV=2.9, MIN/MAX_RADIUS_CROWN=27/28, COG_X=60, COG_Y=80)
%
% Coordinate convention (atan2):
%   atan2(delta_row, delta_col) - first arg is row difference
%   angle=0: right, angle=pi/2: down, positive = clockwise (image frame)

    persistent arc_distances arc_angles
    if isempty(arc_distances)
        % Pre-compute distance and angle grids (once, then cached)
        [M, N] = meshgrid(1:FRAME_SIZE_WIDTH, 1:FRAME_SIZE_HEIGHT);
        arc_distances = sqrt((N - COG_X).^2 + (M - COG_Y).^2);
        arc_angles = atan2(N - COG_X, M - COG_Y);
    end

    % Radius condition: thin annulus 27 <= dist <= 28 (1-pixel thick ring)
    radius_mask = (arc_distances >= MIN_RADIUS_CROWN) & ...
                  (arc_distances <= MAX_RADIUS_CROWN);

    % Angle condition: restrict to FOV around previous VTP direction
    if isnan(theta_prev)
        % Initial state: search full 360 degrees
        angle_mask = true(FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);
    else
        half_fov = pi / FOV;  % pi/2.9 ~ 1.08 rad ~ 62 degrees
        % Wrap angle difference to [-pi, pi]
        angle_diff = mod(arc_angles - theta_prev + pi, 2*pi) - pi;
        angle_mask = abs(angle_diff) <= half_fov;
    end

    % Combined Arc Mask
    arc_mask = radius_mask & angle_mask;
    masked = IMG_eroded & arc_mask;

    % Extract VTP: arithmetic mean of positive pixels
    [vtp_rows, vtp_cols] = find(masked);
    if numel(vtp_rows) > 0
        x_VTP = mean(vtp_rows);
        y_VTP = mean(vtp_cols);
        ex = x_VTP - COG_X;
        ey = y_VTP - COG_Y;
        theta_next = atan2(ex, ey);  % Update direction toward new VTP
        Flag_VTP = true;
    else
        ex = 0;
        ey = 0;
        theta_next = theta_prev;  % Keep previous direction
        Flag_VTP = false;
    end
end
