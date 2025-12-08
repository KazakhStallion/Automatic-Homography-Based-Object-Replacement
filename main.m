clear; close all; clc;

% Load images
scene  = imread('scene4.png');
poster = imread('poster.png');

% Ensure 3 channels
if size(scene,3)  == 1, scene  = repmat(scene,  [1 1 3]); end
if size(poster,3) == 1, poster = repmat(poster, [1 1 3]); end

% Detect plane + Homography
[H, quadPts, debug] = detect_plane_and_homography(scene, poster);

sceneH = size(scene,1);
sceneW = size(scene,2);

% Convex hull
hull = debug.hull;   % Mx2 [x y]

% Corners selected from hull
cornerPts = quadPts;   % your ordered TL,TR,BR,BL

% FIGURE 1: Hull + Quad
figure('Name','Hull + Quad','Color','w');
imshow(scene); hold on;

% Plot convex hull (yellow)
hHull = plot([hull(:,1); hull(1,1)], ...
             [hull(:,2); hull(1,2)], 'y-', 'LineWidth', 3);

% Plot final quadrilateral (green)
hQuad = plot([quadPts(:,1); quadPts(1,1)], ...
             [quadPts(:,2); quadPts(1,2)], 'g-', 'LineWidth', 3);

% Plot corner points (red)
plot(quadPts(:,1), quadPts(:,2), 'ro', ...
     'MarkerSize', 10, 'LineWidth', 2);

% Proper legend (black text, uses only the 2 line handles)
legend([hHull hQuad], {'Hull','Quad'}, ...
       'Location','southeast', ...
       'TextColor','k');

title('Hull Outline + Detected Quad', 'FontSize', 16);
hold off;


% Warp + Composite
[outImg, warpedPoster, mask] = warp_and_composite(scene, poster, H);


% FIGURE 2: Summary
figure('Name','Pipeline Overview','Color','w');

tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% (1) Original scene + quad
nexttile;
imshow(scene); hold on;
plot([quadPts(:,1); quadPts(1,1)], ...
     [quadPts(:,2); quadPts(1,2)], 'g-', 'LineWidth', 2);
plot(quadPts(:,1), quadPts(:,2), 'ro', 'LineWidth', 2);
title('Original Scene + Detected Quad');

% (2) Original poster
nexttile;
imshow(poster);
title('Original Poster');

% (3) Warped poster
nexttile;
imshow(warpedPoster);
title('Warped Poster (H Applied)');

% (4) Final AR composite
nexttile;
imshow(outImg);
title('Final AR Composite');