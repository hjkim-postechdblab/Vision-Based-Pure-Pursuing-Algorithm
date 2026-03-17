function [IMG_eroded, IMG_marker_eroded] = erodeTrackAndMarker(Fbin, ...
    SQUARE_KERNEL, DISK_KERNEL)
%erodeTrackAndMarker  Apply square and disk erosion to binary frame
%   Fbin: logical H x W (binarized frame)
%   IMG_eroded: logical H x W - square erosion for track following
%   IMG_marker_eroded: logical H x W - disk erosion for marker detection
%
% Reference: tech_spec.md Section 3.3
% Parameters: variables.m (SQUARE_KERNEL=8, DISK_KERNEL=11)
%
% Note: strel/imerode may not support code generation.
% If codegen fails, replace with manual min-filter implementation below.

    % Track following erosion: square 8x8 kernel
    se_square = strel('square', SQUARE_KERNEL);
    IMG_eroded = imerode(Fbin, se_square);

    % Marker detection erosion: disk kernel radius 11
    % Thin track lines are destroyed; circular marker survives
    se_disk = strel('disk', DISK_KERNEL);
    IMG_marker_eroded = imerode(Fbin, se_disk);
end

%% Manual erosion fallback (for Simulink code generation)
% Uncomment and use if strel/imerode are not supported for codegen:
%
% function IMG_out = manualErode(Fbin, kernel)
%     [H, W] = size(Fbin);
%     [kH, kW] = size(kernel);
%     padH = floor(kH/2);
%     padW = floor(kW/2);
%     IMG_out = false(H, W);
%     for n = (1+padH):(H-padH)
%         for m = (1+padW):(W-padW)
%             patch = Fbin((n-padH):(n+padH), (m-padW):(m+padW));
%             IMG_out(n,m) = all(patch(kernel));
%         end
%     end
% end
