%% integrateFCS.m - Integrate IPS and PP blocks into Flight Control System
%
% Run this script in MATLAB after opening the project:
%   1. Open parrotMinidroneCompetition.prj
%   2. Run this script from the MATLAB command window
%
% This script adds IPS and PathPlanner MATLAB Function blocks to
% flightControlSystem.slx and wires the signal connections.
%
% Reference: docs/Phase1/tech_spec.md Section 1.1, 1.3, 1.4
%            docs/Phase1/plan.md Step 9
%
% Prerequisites:
%   - IPS/*.m files (Step 1~5 completed)
%   - PP/PathPlanner.m (Step 6~8 completed)
%   - variables.m has MARKER_MIN_PIXELS parameter

%% ---- Configuration ----
modelName = 'flightControlSystem';

fprintf('=== Phase 1: Integrating IPS and PP into %s ===\n', modelName);

%% ---- Step 1: Open the model ----
fprintf('[1/6] Opening model...\n');
open_system(modelName);

%% ---- Step 2: Add IPS MATLAB Function Block ----
fprintf('[2/6] Adding IPS block...\n');

ipsBlockPath = [modelName '/IPS'];
if ~exist_block(ipsBlockPath)
    add_block('simulink/User-Defined Functions/MATLAB Function', ipsBlockPath);
    set_param(ipsBlockPath, 'Position', [200 100 350 200]);
    fprintf('  IPS block added. MANUAL ACTION REQUIRED:\n');
    fprintf('  -> Double-click the IPS block and paste contents of IPS/IPS.m\n');
    fprintf('  -> Set Sample Time to VTs (0.2s)\n');
else
    fprintf('  IPS block already exists, skipping.\n');
end

%% ---- Step 3: Add PathPlanner MATLAB Function Block ----
fprintf('[3/6] Adding PathPlanner block...\n');

ppBlockPath = [modelName '/PathPlanner'];
if ~exist_block(ppBlockPath)
    add_block('simulink/User-Defined Functions/MATLAB Function', ppBlockPath);
    set_param(ppBlockPath, 'Position', [500 100 700 250]);
    fprintf('  PathPlanner block added. MANUAL ACTION REQUIRED:\n');
    fprintf('  -> Double-click the PathPlanner block and paste contents of PP/PathPlanner.m\n');
    fprintf('  -> Set Sample Time to Ts (0.005s)\n');
else
    fprintf('  PathPlanner block already exists, skipping.\n');
end

%% ---- Step 4: Add Rate Transition Block ----
fprintf('[4/6] Adding Rate Transition (IPS->PP)...\n');

rtBlockPath = [modelName '/RateTransition_IPS_PP'];
if ~exist_block(rtBlockPath)
    add_block('simulink/Signal Attributes/Rate Transition', rtBlockPath);
    set_param(rtBlockPath, 'Position', [380 130 450 170]);
    fprintf('  Rate Transition block added (Zero-Order Hold: IPS 5Hz -> PP 200Hz).\n');
else
    fprintf('  Rate Transition block already exists, skipping.\n');
end

%% ---- Step 5: Signal Connection Guide ----
fprintf('[5/6] Signal connection guide:\n');
fprintf('\n');
fprintf('  === IPS Block Connections ===\n');
fprintf('  INPUT:  IMG (uint8 H x W x 3) <- Camera sensor output\n');
fprintf('  OUTPUT: ex (double 1x1)        -> Rate Transition -> PP\n');
fprintf('  OUTPUT: ey (double 1x1)        -> Rate Transition -> PP\n');
fprintf('  OUTPUT: Flag_VTP (boolean 1x1) -> Rate Transition -> PP\n');
fprintf('  OUTPUT: Flag_marker (bool 1x1) -> Rate Transition -> PP\n');
fprintf('\n');
fprintf('  === PathPlanner Block Connections ===\n');
fprintf('  INPUT:  ex, ey, Flag_VTP, Flag_marker <- Rate Transition (from IPS)\n');
fprintf('  INPUT:  current_z (double)     <- Drone state (altitude, NED)\n');
fprintf('  INPUT:  ips_updated (boolean)  <- Detect IPS output change\n');
fprintf('  OUTPUT: xw (double)            -> Position controller (x command)\n');
fprintf('  OUTPUT: yw (double)            -> Position controller (y command)\n');
fprintf('  OUTPUT: zw (double)            -> Position controller (z command)\n');
fprintf('\n');
fprintf('  === ips_updated Signal ===\n');
fprintf('  Use a Detect Change block on any IPS output (e.g., ex) to generate\n');
fprintf('  ips_updated boolean at PP rate. Or use a counter-based approach:\n');
fprintf('  ips_updated = (mod(step_count, VTs/Ts) == 0)\n');

%% ---- Step 6: Verify ----
fprintf('[6/6] Verification checklist:\n');
fprintf('  [ ] IPS block: Sample Time = VTs = 0.2\n');
fprintf('  [ ] PP block: Sample Time = Ts = 0.005\n');
fprintf('  [ ] Rate Transition between IPS and PP outputs\n');
fprintf('  [ ] Camera input connected to IPS\n');
fprintf('  [ ] PP outputs connected to position controller\n');
fprintf('  [ ] current_z from drone state connected to PP\n');
fprintf('  [ ] Build: Code Generation button -> 0 errors\n');
fprintf('  [ ] commandVars.m: Command.yawStepAmplitude = 0 (heading-free)\n');
fprintf('\n');
fprintf('=== Integration guide complete. Follow MANUAL ACTIONs above. ===\n');

%% ---- Helper Function ----
function exists = exist_block(blockPath)
    try
        get_param(blockPath, 'BlockType');
        exists = true;
    catch
        exists = false;
    end
end
