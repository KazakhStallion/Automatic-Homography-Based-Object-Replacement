clear; close all; clc;

% Input images
scene  = imread('scene.png');
poster = imread('poster.png');

% Ensure RGB
if size(scene,3) == 1,  scene  = repmat(scene, [1 1 3]);  end
if size(poster,3) == 1, poster = repmat(poster,[1 1 3]); end

[H, quadPts, debug] = detect_plane_and_homography(scene, poster);

figure; imshow(scene); title('Detected Quad');
hold on;
plot([quadPts(:,1); quadPts(1,1)], ...
     [quadPts(:,2); quadPts(1,2)], 'g-', 'LineWidth', 3);
plot(quadPts(:,1), quadPts(:,2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
hold off;

fprintf('Homography H:\n');
disp(H);

% Warp and composite
[outImg, warpedPoster, mask] = warp_and_composite(scene, poster, H);

figure; 
subplot(1,4,1); imshow(scene); title('Original Scene');
hold on; plot([quadPts(:,1); quadPts(1,1)], [quadPts(:,2); quadPts(1,2)], 'g-', 'LineWidth', 2);
subplot(1,4,2); imshow(poster); title('Original Poster');
subplot(1,4,3); imshow(warpedPoster); title('Warped Poster');
subplot(1,4,4); imshow(outImg); title('Final AR Composite');

% Edge map & candidate mask
figure; 
subplot(1,2,1); imshow(debug.edge); title('Canny Edges');
subplot(1,2,2); imshow(debug.candidateMask); title('Selected White Region');