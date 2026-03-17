function [xw, yw, zw] = PathPlanner(ex, ey, Flag_VTP, Flag_marker, ...
                                     current_z, ips_updated)
%PathPlanner  Path Planner - error signals to position commands
%   Runs at 200Hz (T_PP=0.005s). IPS provides inputs at 5Hz (T_IPS=0.2s).
%
%   Inputs:
%     ex, ey: double - image error from IPS [pixel]
%     Flag_VTP: boolean - track detected
%     Flag_marker: boolean - landing marker detected
%     current_z: double - current altitude [m, NED]
%     ips_updated: boolean - true when IPS has new data this step
%
%   Outputs:
%     xw, yw, zw: double - position commands [m, NED]
%
% Reference: tech_spec.md Section 2 (State Machine), 4 (PP detail)
% Parameters: variables.m (GAIN_TRACK, GAIN_LANDING, Z_LOW, Z_HIGH, etc.)
%             startVars.m (Ts=0.005)
%
% State Machine:
%   S1(TAKEOFF) -> hovering_ok & Flag_VTP -> S2(FOLLOWING)
%   S2(FOLLOWING) -> ~Flag_VTP & Flag_marker -> S3(END_MARKER)
%   S3(END_MARKER) -> centered_ok -> S4(LANDING)

    persistent x_prev y_prev mission_state hover_counter center_counter
    persistent deriv_hold_counter track_lost_counter
    persistent ex_prev_pp ey_prev_pp delta_ex delta_ey

    % Initialization (tech_spec Section 1.3 PP block)
    if isempty(mission_state)
        mission_state = uint8(1);       % S1: TAKEOFF
        x_prev = 0;                     % Set from init.posNED(1) at integration
        y_prev = 0;                     % Set from init.posNED(2) at integration
        hover_counter = uint32(0);
        center_counter = uint32(0);
        deriv_hold_counter = uint32(0);
        track_lost_counter = uint32(0);
        ex_prev_pp = 0;
        ey_prev_pp = 0;
        delta_ex = 0;
        delta_ey = 0;
    end

    % ---- Derivative computation (only at IPS update, 5Hz) ----
    % Reference: tech_spec.md Section 4.4
    if ips_updated
        delta_ex = ex - ex_prev_pp;
        delta_ey = ey - ey_prev_pp;
        delta_norm = sqrt(delta_ex^2 + delta_ey^2);
        if delta_norm > CHANGE_DERIVATIVE_ERROR_THRESHOLD  % > 2 pixel
            deriv_hold_counter = uint32(TIME_HOLD / Ts);   % 0.2/0.005 = 40 steps
        end
        ex_prev_pp = ex;
        ey_prev_pp = ey;
    end

    % ---- State Machine ----
    switch mission_state

        case 1  % S1: TAKEOFF
            % Ascend to target altitude, wait for hover stabilization
            % Reference: tech_spec.md Section 2.2 (S1->S2)
            xw = x_prev;
            yw = y_prev;
            zw = Z_LOW;  % = -1 (target altitude, NED)

            % Hovering check: Z_HIGH(-1.2) <= current_z <= Z_LOW(-1)
            % NED: more negative = higher, so Z_HIGH < Z_LOW numerically
            if (current_z >= Z_HIGH) && (current_z <= Z_LOW) && Flag_VTP
                hover_counter = hover_counter + 1;
                % HOVER_CONFIRM_FRAMES = T_IPS/T_PP = 0.2/0.005 = 40
                if hover_counter >= uint32(40)
                    mission_state = uint8(2);  % -> S2: FOLLOWING
                end
            else
                hover_counter = uint32(0);  % Reset if out of range
            end

        case 2  % S2: FOLLOWING
            % Track following using VTP error
            % Reference: tech_spec.md Section 4.2, 2.3
            if Flag_VTP
                track_lost_counter = uint32(0);

                % Position update: x_{k+1} = x_k + alpha * ex
                xw = x_prev + GAIN_TRACK * ex;   % GAIN_TRACK = 0.0038
                yw = y_prev + GAIN_TRACK * ey;

                % Derivative part: suppress overshoot at corners
                % Reference: tech_spec.md Section 4.4
                if deriv_hold_counter > 0
                    xw = xw + DERIVATIVE_GAIN * delta_ex;  % DERIVATIVE_GAIN = 0.002
                    yw = yw + DERIVATIVE_GAIN * delta_ey;
                    deriv_hold_counter = deriv_hold_counter - 1;
                end
                zw = Z_LOW;

            elseif Flag_marker
                % S2 -> S3: Track ended, marker detected
                % Reference: tech_spec.md Section 2.2 (~Flag_VTP & Flag_marker)
                mission_state = uint8(3);
                center_counter = uint32(0);
                xw = x_prev + GAIN_LANDING * ex;  % GAIN_LANDING = 0.0022
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;

            else
                % Track lost: hold position
                % Reference: tech_spec.md Section 2.3
                track_lost_counter = track_lost_counter + 1;
                xw = x_prev;
                yw = y_prev;
                zw = Z_LOW;
            end

        case 3  % S3: END_MARKER
            % Align with landing marker center
            % Reference: tech_spec.md Section 4.3
            error_norm = sqrt(ex^2 + ey^2);

            if error_norm <= MAX_ERROR_LANDING  % <= 5 pixel
                center_counter = center_counter + 1;
                xw = x_prev;   % Hold horizontal position
                yw = y_prev;
                zw = Z_LOW;    % Maintain altitude (not landing yet)

                % CENTER_CONFIRM_FRAMES = DELAY_LANDING / Ts = 4/0.005 = 800
                if center_counter >= uint32(DELAY_LANDING / Ts)
                    mission_state = uint8(4);  % -> S4: LANDING
                end
            else
                center_counter = uint32(0);  % Reset on error exceedance
                xw = x_prev + GAIN_LANDING * ex;
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;
            end

        case 4  % S4: LANDING
            % Descend to ground
            % Reference: tech_spec.md Section 2.1
            xw = x_prev;   % Hold horizontal position
            yw = y_prev;
            zw = 0;         % NED z=0 -> ground level landing command

        otherwise
            xw = x_prev;
            yw = y_prev;
            zw = Z_LOW;
    end

    % Update previous position for next step
    x_prev = xw;
    y_prev = yw;
end
