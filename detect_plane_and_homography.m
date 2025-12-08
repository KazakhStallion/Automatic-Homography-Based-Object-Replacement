function [H, quadPts, debug] = detect_plane_and_homography(sceneRGB, posterRGB)
% Detects a white planar quad and computes H: poster -> scene

sceneGray   = rgb2gray(sceneRGB);
sceneDouble = im2double(sceneGray);

% --- 1. Canny edges (debug only) ---
edgeMap = edge(sceneGray, 'canny');

% --- 2. Whiteness mask ---
thBright    = graythresh(sceneGray);          % Otsu baseline
brightMask  = sceneDouble > max(thBright,0.7);
brightMask  = bwareaopen(brightMask, 500);    % remove tiny blobs
brightMask  = imfill(brightMask, 'holes');    % fill holes

% --- 3. Pick largest bright region (paper) ---
stats = regionprops(brightMask, 'Area','PixelIdxList');
if isempty(stats)
    error('No bright regions found. Adjust threshold or lighting.');
end

[~, idxMax] = max([stats.Area]);
bestRegion  = stats(idxMax);

candidateMask = false(size(brightMask));
candidateMask(bestRegion.PixelIdxList) = true;

% --- 4. Convex hull of the paper region ---
sHull = regionprops(candidateMask, 'ConvexHull');
hull  = sHull.ConvexHull;   % Nx2, [x y] around the paper

% Safety: need enough hull points
if size(hull,1) < 4
    error('Convex hull has fewer than 4 points.');
end

% --- 5. Cluster hull into 4 corner groups with k-means ---
% (Statistics and Machine Learning Toolbox)
[idx,C] = kmeans(hull, 4, 'Replicates', 5);

quadPts = C;    % 4×2, each row is approx corner [x y]

% Order as TL, TR, BR, BL
quadPts = order_quad_points(quadPts);

% --- 6. Homography from poster corners to quadPts ---
[H, ~] = build_homography_from_poster(posterRGB, quadPts);

% --- 7. Debug outputs ---
debug.edge          = edgeMap;
debug.candidateMask = candidateMask;
debug.hull          = hull;
debug.quadPts       = quadPts;
debug.clusterIdx    = idx;
end
