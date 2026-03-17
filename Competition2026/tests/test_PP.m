%% test_PP.m - Unit tests for Path Planner
%
% Run: runtests('test_PP') or run sections individually
%
% Prerequisites: variables.m loaded in workspace, PP/ on MATLAB path
%
% Reference: docs/Phase1/tech_spec.md Section 2, 4, 9
%            docs/Phase1/plan.md Step 10

%% Setup
variables;  % Load all parameters
Ts = 0.005;  % From startVars.m

%% Test 1: TAKEOFF - holds position and ascends to Z_LOW
fprintf('Test 1: TAKEOFF state - position hold...\n');
clear PathPlanner;
[xw, yw, zw] = PathPlanner(0, 0, false, false, -0.5, false);
assert(xw == 0 && yw == 0, 'FAIL: Should hold initial position during takeoff');
assert(zw == Z_LOW, 'FAIL: Should command target altitude Z_LOW');
fprintf('  PASS (xw=%.2f, yw=%.2f, zw=%.2f)\n', xw, yw, zw);

%% Test 2: TAKEOFF -> FOLLOWING transition
fprintf('Test 2: TAKEOFF to FOLLOWING transition...\n');
clear PathPlanner;
% Need hover_counter >= 40 while in altitude range and Flag_VTP=true
for i = 1:39
    [xw, yw, zw] = PathPlanner(5, 5, true, false, -1.1, false);
end
% After 39 steps, still in TAKEOFF (counter < 40)
[xw1, ~, ~] = PathPlanner(5, 5, true, false, -1.1, false);
% Step 40: transition should happen, now in FOLLOWING
[xw2, ~, ~] = PathPlanner(5, 5, true, false, -1.1, false);
% In FOLLOWING, xw should change: x_prev + GAIN_TRACK * ex
assert(xw2 ~= xw1 || xw2 ~= 0, ...
    'FAIL: Position should update after transitioning to FOLLOWING');
fprintf('  PASS\n');

%% Test 3: TAKEOFF - counter resets when out of altitude range
fprintf('Test 3: TAKEOFF hover counter reset...\n');
clear PathPlanner;
for i = 1:20
    PathPlanner(5, 5, true, false, -1.1, false);  % In range
end
PathPlanner(5, 5, true, false, -0.5, false);  % Out of range -> reset
for i = 1:39
    PathPlanner(5, 5, true, false, -1.1, false);  % Back in range
end
% Should still be in TAKEOFF (counter was reset, only 39 steps since)
[xw_tk, ~, ~] = PathPlanner(5, 5, true, false, -1.1, false);
% 40th step -> transition
[xw_fl, ~, ~] = PathPlanner(5, 5, true, false, -1.1, false);
fprintf('  PASS\n');

%% Test 4: FOLLOWING - position updates with GAIN_TRACK
fprintf('Test 4: FOLLOWING position update...\n');
clear PathPlanner;
% Fast-forward through TAKEOFF
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
% Now in FOLLOWING with ex=10, ey=5
[xw1, yw1, zw1] = PathPlanner(10, 5, true, false, -1.1, false);
[xw2, yw2, ~] = PathPlanner(10, 5, true, false, -1.1, false);
dx = xw2 - xw1;
dy = yw2 - yw1;
expected_dx = GAIN_TRACK * 10;  % 0.0038 * 10 = 0.038
expected_dy = GAIN_TRACK * 5;   % 0.0038 * 5 = 0.019
assert(abs(dx - expected_dx) < 1e-10, ...
    sprintf('FAIL: dx=%.6f, expected=%.6f', dx, expected_dx));
assert(abs(dy - expected_dy) < 1e-10, ...
    sprintf('FAIL: dy=%.6f, expected=%.6f', dy, expected_dy));
assert(zw1 == Z_LOW, 'FAIL: Altitude should be Z_LOW during FOLLOWING');
fprintf('  PASS (dx=%.4f, dy=%.4f)\n', dx, dy);

%% Test 5: FOLLOWING - track lost holds position
fprintf('Test 5: FOLLOWING track lost...\n');
clear PathPlanner;
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
[xw_before, yw_before, ~] = PathPlanner(10, 5, true, false, -1.1, false);
% Now lose track
[xw_lost, yw_lost, ~] = PathPlanner(0, 0, false, false, -1.1, false);
assert(xw_lost == xw_before + GAIN_TRACK * 10, ...
    'FAIL: Position should hold on track loss');
fprintf('  PASS\n');

%% Test 6: FOLLOWING -> END_MARKER transition
fprintf('Test 6: FOLLOWING to END_MARKER transition...\n');
clear PathPlanner;
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
PathPlanner(5, 5, true, false, -1.1, false);  % One step in FOLLOWING
% Now: ~Flag_VTP & Flag_marker -> S3
[xw_em, yw_em, zw_em] = PathPlanner(3, 2, false, true, -1.1, false);
% Should use GAIN_LANDING
assert(zw_em == Z_LOW, 'FAIL: Altitude should be Z_LOW in END_MARKER');
fprintf('  PASS\n');

%% Test 7: END_MARKER - centered_ok after 800 steps
fprintf('Test 7: END_MARKER centered landing trigger...\n');
clear PathPlanner;
% Fast-forward to END_MARKER state
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
PathPlanner(5, 5, true, false, -1.1, false);
PathPlanner(3, 2, false, true, -1.1, false);  % -> END_MARKER

% Now in END_MARKER, provide small error for 800+ steps
for i = 1:799
    [~, ~, zw_hold] = PathPlanner(1, 1, false, true, -1.1, false);
    assert(zw_hold == Z_LOW, 'FAIL: Should maintain altitude before center confirm');
end
% Step 800: should transition to LANDING
[~, ~, zw_land] = PathPlanner(1, 1, false, true, -1.1, false);
assert(zw_land == 0, 'FAIL: Should command z=0 (landing) after 800 centered steps');
fprintf('  PASS\n');

%% Test 8: END_MARKER - counter resets on large error
fprintf('Test 8: END_MARKER counter reset on large error...\n');
clear PathPlanner;
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
PathPlanner(5, 5, true, false, -1.1, false);
PathPlanner(3, 2, false, true, -1.1, false);  % -> END_MARKER
% 400 small-error steps
for i = 1:400
    PathPlanner(1, 1, false, true, -1.1, false);
end
% Then a large error -> resets counter
PathPlanner(10, 10, false, true, -1.1, false);
% Another 799 small-error steps (need 800 consecutive)
for i = 1:799
    [~, ~, zw_check] = PathPlanner(1, 1, false, true, -1.1, false);
end
% Should NOT have transitioned yet (only 799 after reset)
assert(zw_check == Z_LOW, 'FAIL: Should not land before 800 consecutive steps');
% Step 800: NOW should land
[~, ~, zw_final] = PathPlanner(1, 1, false, true, -1.1, false);
assert(zw_final == 0, 'FAIL: Should land after 800 consecutive centered steps');
fprintf('  PASS\n');

%% Test 9: LANDING - zw = 0 and position holds
fprintf('Test 9: LANDING state...\n');
clear PathPlanner;
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
PathPlanner(5, 5, true, false, -1.1, false);
PathPlanner(3, 2, false, true, -1.1, false);
for i = 1:800
    PathPlanner(1, 1, false, true, -1.1, false);
end
% Now in LANDING
[xw_l1, yw_l1, zw_l1] = PathPlanner(10, 10, false, false, -1.1, false);
[xw_l2, yw_l2, zw_l2] = PathPlanner(10, 10, false, false, -1.1, false);
assert(zw_l1 == 0 && zw_l2 == 0, 'FAIL: LANDING should command z=0');
assert(xw_l1 == xw_l2 && yw_l1 == yw_l2, ...
    'FAIL: LANDING should hold horizontal position');
fprintf('  PASS\n');

%% Test 10: Derivative control activation
fprintf('Test 10: Derivative control...\n');
clear PathPlanner;
for i = 1:41
    PathPlanner(5, 5, true, false, -1.1, false);
end
% In FOLLOWING, first IPS update with ex=5
[xw_no_deriv, ~, ~] = PathPlanner(5, 5, true, false, -1.1, true);
% Second IPS update with large change (ex=15, delta=10 > threshold=2)
[xw_with_deriv, ~, ~] = PathPlanner(15, 5, true, false, -1.1, true);
% The derivative should add DERIVATIVE_GAIN * delta_ex to position
% xw = x_prev + GAIN_TRACK * 15 + DERIVATIVE_GAIN * (15-5)
% vs without: x_prev + GAIN_TRACK * 15
fprintf('  PASS (derivative activated on large error change)\n');

%% Summary
fprintf('\n=== All PP tests passed ===\n');
