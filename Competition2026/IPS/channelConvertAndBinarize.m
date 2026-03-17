function Fbin = channelConvertAndBinarize(IMG, G_B_GAIN, BINARIZER_THRESHOLD)
%channelConvertAndBinarize  RGB -> red-enhanced binarization
%   IMG: uint8 H x W x 3 RGB frame
%   Fbin: logical H x W binary result
%
% Reference: tech_spec.md Section 3.1, 3.2
% Parameters: variables.m (G_B_GAIN=2, BINARIZER_THRESHOLD=100)

    % Step 1: Channel conversion - subtract G/B from R to enhance red
    F = double(IMG(:,:,1)) - double(IMG(:,:,2))/G_B_GAIN ...
                            - double(IMG(:,:,3))/G_B_GAIN;

    % Step 2: Binarization - pixels above threshold are track candidates
    Fbin = F >= BINARIZER_THRESHOLD;
end
