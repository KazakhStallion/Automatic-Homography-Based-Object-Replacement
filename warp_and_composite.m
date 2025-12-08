function [outImg, warpedPoster, mask] = warp_and_composite(sceneRGB, posterRGB, H)
% Warps poster into scene using homography H (poster -> scene).

[Hscene, Wscene, ~] = size(sceneRGB);

% Projective transform object expects transpose of H
tform = projective2d(H');

% Warp poster
refScene = imref2d([Hscene, Wscene]);
[warpedPoster, ~] = imwarp(posterRGB, tform, 'OutputView', refScene);

% Warp a binary mask of poster
posterMask = true(size(posterRGB,1), size(posterRGB,2));
[mask, ~] = imwarp(posterMask, tform, 'OutputView', refScene);

% Composite: simple overwrite inside mask
outImg = sceneRGB;
for c = 1:3
    channel = outImg(:,:,c);
    wChan   = warpedPoster(:,:,c);
    channel(mask) = wChan(mask);
    outImg(:,:,c) = channel;
end

% Optional: soft edge blending (feathering) – small blur on mask
% Uncomment if you want smoother transition:
se = strel('disk',3);
softMask = imerode(mask, se);
alpha = imgaussfilt(double(softMask), 2);
alpha = alpha / max(alpha(:));
for c = 1:3
    channel = double(sceneRGB(:,:,c));
    wChan   = double(warpedPoster(:,:,c));
    outImg(:,:,c) = uint8(alpha .* wChan + (1-alpha) .* channel);
end
end