%% test_IPS.m - Unit tests for Image Processing System
%
% Run: runtests('test_IPS') or run sections individually
%
% Prerequisites: variables.m loaded in workspace, IPS/ on MATLAB path
%
% Reference: docs/Phase1/tech_spec.md Section 3, 9
%            docs/Phase1/plan.md Step 10

%% Setup
variables;  % Load all parameters

%% Test 1: channelConvertAndBinarize - red track detected
fprintf('Test 1: Red track binarization...\n');
IMG_red = uint8(zeros(120, 160, 3));
IMG_red(50:70, 70:90, 1) = 255;  % Red block at center
Fbin = channelConvertAndBinarize(IMG_red, G_B_GAIN, BINARIZER_THRESHOLD);
assert(sum(Fbin(:)) > 0, 'FAIL: Red region should produce positive pixels');
assert(Fbin(60, 80) == true, 'FAIL: Center red pixel should be true');
fprintf('  PASS\n');

%% Test 2: channelConvertAndBinarize - grey background rejected
fprintf('Test 2: Grey background rejection...\n');
IMG_grey = uint8(128 * ones(120, 160, 3));  % Uniform grey
Fbin_grey = channelConvertAndBinarize(IMG_grey, G_B_GAIN, BINARIZER_THRESHOLD);
assert(sum(Fbin_grey(:)) == 0, 'FAIL: Grey image should produce no positive pixels');
fprintf('  PASS\n');

%% Test 3: channelConvertAndBinarize - green/blue rejected
fprintf('Test 3: Green/blue rejection...\n');
IMG_green = uint8(zeros(120, 160, 3));
IMG_green(:,:,2) = 255;  % All green
Fbin_green = channelConvertAndBinarize(IMG_green, G_B_GAIN, BINARIZER_THRESHOLD);
assert(sum(Fbin_green(:)) == 0, 'FAIL: Green image should produce no positive pixels');
fprintf('  PASS\n');

%% Test 4: erodeTrackAndMarker - thin line removed by disk, kept by square
fprintf('Test 4: Erosion behavior...\n');
Fbin_line = false(120, 160);
Fbin_line(60, 40:120) = true;  % 1-pixel thin horizontal line
[IMG_sq, IMG_disk] = erodeTrackAndMarker(Fbin_line, SQUARE_KERNEL, DISK_KERNEL);
assert(sum(IMG_sq(:)) == 0, 'FAIL: 1px line should be eroded by square kernel');
assert(sum(IMG_disk(:)) == 0, 'FAIL: 1px line should be eroded by disk kernel');
fprintf('  PASS\n');

%% Test 5: erodeTrackAndMarker - large circle survives disk erosion
fprintf('Test 5: Large circle survives disk erosion...\n');
Fbin_circle = false(120, 160);
for n = 1:120
    for m = 1:160
        if sqrt((n-60)^2 + (m-80)^2) <= 20
            Fbin_circle(n,m) = true;
        end
    end
end
[~, IMG_disk_circle] = erodeTrackAndMarker(Fbin_circle, SQUARE_KERNEL, DISK_KERNEL);
assert(sum(IMG_disk_circle(:)) > 0, ...
    'FAIL: Circle radius 20 should survive disk erosion (kernel=11)');
fprintf('  PASS\n');

%% Test 6: findVTP - 360 degree search when theta_prev=NaN
fprintf('Test 6: VTP 360-degree initial search...\n');
% Create eroded image with pixels on the arc ring (radius 27-28)
IMG_arc = false(120, 160);
for n = 1:120
    for m = 1:160
        dist = sqrt((n - COG_X)^2 + (m - COG_Y)^2);
        if dist >= 27 && dist <= 28 && m > COG_Y  % Right half
            IMG_arc(n, m) = true;
        end
    end
end
clear findVTP;  % Reset persistent variables
[ex_v, ey_v, Flag_v, theta_v] = findVTP(IMG_arc, NaN, ...
    MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y, ...
    FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);
assert(Flag_v == true, 'FAIL: VTP should be detected on arc');
assert(ey_v > 0, 'FAIL: VTP should be to the right (positive ey)');
assert(~isnan(theta_v), 'FAIL: theta should be updated after detection');
fprintf('  PASS (ex=%.1f, ey=%.1f, theta=%.2f)\n', ex_v, ey_v, theta_v);

%% Test 7: findVTP - FOV restriction with valid theta_prev
fprintf('Test 7: VTP FOV restriction...\n');
% With theta_prev pointing right (0 rad), left-side pixels should be excluded
IMG_left = false(120, 160);
for n = 1:120
    for m = 1:160
        dist = sqrt((n - COG_X)^2 + (m - COG_Y)^2);
        if dist >= 27 && dist <= 28 && m < COG_Y - 20  % Far left
            IMG_left(n, m) = true;
        end
    end
end
clear findVTP;
[~, ~, Flag_left, ~] = findVTP(IMG_left, 0, ...  % theta_prev = 0 (right)
    MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y, ...
    FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);
assert(Flag_left == false, 'FAIL: Left-side pixels should be outside FOV when facing right');
fprintf('  PASS\n');

%% Test 8: findMarker - minimum pixel threshold
fprintf('Test 8: Marker minimum pixel threshold...\n');
IMG_small_blob = false(120, 160);
IMG_small_blob(60, 80:82) = true;  % Only 3 pixels (< MARKER_MIN_PIXELS=10)
[~, ~, Flag_small] = findMarker(IMG_small_blob, COG_X, COG_Y, MARKER_MIN_PIXELS);
assert(Flag_small == false, 'FAIL: Small blob should not be detected as marker');

IMG_big_blob = false(120, 160);
for n = 55:65
    for m = 75:85
        IMG_big_blob(n, m) = true;  % 11x11 = 121 pixels
    end
end
[ex_m, ey_m, Flag_big] = findMarker(IMG_big_blob, COG_X, COG_Y, MARKER_MIN_PIXELS);
assert(Flag_big == true, 'FAIL: Large blob should be detected as marker');
assert(abs(ex_m) < 1 && abs(ey_m) < 1, ...
    'FAIL: Centered marker should have near-zero error');
fprintf('  PASS\n');

%% Test 9: IPS integration - Flag mutual exclusivity
fprintf('Test 9: IPS flag mutual exclusivity...\n');
% Create image with both track arc and large circle
IMG_both = uint8(zeros(120, 160, 3));
% Add red arc (track)
for n = 1:120
    for m = 1:160
        dist = sqrt((n-60)^2 + (m-80)^2);
        if (dist >= 20 && dist <= 40) && m > 80  % Red arc right side
            IMG_both(n, m, 1) = 255;
        end
    end
end
clear IPS findVTP;  % Reset persistent
[~, ~, fv, fm] = IPS(IMG_both);
assert(~(fv && fm), 'FAIL: Flag_VTP and Flag_marker must be mutually exclusive');
fprintf('  PASS (Flag_VTP=%d, Flag_marker=%d)\n', fv, fm);

%% Summary
fprintf('\n=== All IPS tests passed ===\n');
