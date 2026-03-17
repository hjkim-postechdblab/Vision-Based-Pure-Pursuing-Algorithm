% variables.m - IPS/PP parameters for Competition2026
%
% Adapted from UniSannio_Team/variables.m (IFAC2020) for the 2026
% MathWorks Minidrone Competition model.
%
% Camera frame size: 120 x 160 (same as 2020 model)
%
% TODO (tech_spec Section 7.3): After initial integration, run the 8-step
%   parameter tuning sequence:
%   1. Verify frame size -> COG_X, COG_Y
%   2. Check camera RGB distribution
%   3. Re-tune BINARIZER_THRESHOLD
%   4. Re-tune SQUARE_KERNEL (track width 1cm->10cm)
%   5. Re-tune MIN/MAX_RADIUS_CROWN
%   6. Re-tune DISK_KERNEL + MARKER_MIN_PIXELS (marker 2cm->20cm)
%   7. Re-tune GAIN_TRACK, GAIN_LANDING
%   8. Simple track full pass test

% Copyright 2026

%% IMAGE PROCESSING SYSTEM - Image Binarization Block

G_B_GAIN = 2;
BINARIZER_THRESHOLD = 100;

%% IMAGE PROCESSING SYSTEM - Waypoints Follower block

SQUARE_KERNEL = 8;

FOV = 2.9;  %Field of view= 2*(pi/FOV) this is a portion of the circumference

MIN_RADIUS_CROWN = 27;
MAX_RADIUS_CROWN = 28;

FRAME_SIZE_HEIGHT = 120;
FRAME_SIZE_WIDTH = 160;

COG_X = (FRAME_SIZE_HEIGHT/2);
COG_Y = (FRAME_SIZE_WIDTH/2);

%% IMAGE PROCESSING SYSTEM - Land Marker Detector block

DISK_KERNEL = 11;
MARKER_MIN_PIXELS = 10;   % Min pixels after disk erosion to confirm marker

%% PATH PLANNING SYSTEM - Task Planning block

FLAG_WAIT_FOR_HOVERING = 0;

%% PATH PLANNING SYSTEM - Nonlinear Path Planner/ - X PLANNER & Y PLANNER

GAIN_LANDING = 0.0022;

GAIN_TRACK = 0.0038;

%% PATH PLANNING SYSTEM - Take off checker block

Z_LOW = -1;
Z_HIGH = -1.2;

%% PATH PLANNING SYSTEM - Nonlinear Path Planner - X PLANNER && Y PLANNER - x derivativepart & y derivative part

CHANGE_DERIVATIVE_ERROR_THRESHOLD = 2;
DERIVATIVE_GAIN = 0.002 ;
TIME_HOLD = 0.2;

%% PATH PLANNING SYSTEM - Non linear Path Planner - Z PLANNER

DELAY_LANDING = 4;

MAX_ERROR_LANDING = 5;
