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
hull  = sHull.ConvexHull;   % Mx2, [x y]

M = size(hull,1);
if M < 4
    error('Convex hull has fewer than 4 points.');
end


% 5. Corner detection from hull using turn angle + quadrants

pts = hull;                % Mx2, assume ordered around the boundary

% indices of previous and next points in the cyclic hull
prevIdx = [M, 1:M-1];
nextIdx = [2:M, 1];

v1 = pts - pts(prevIdx,:);       % prev -> current
v2 = pts(nextIdx,:) - pts;       % current -> next

% normalize, avoid division by zero
v1n = sqrt(sum(v1.^2,2)) + eps;
v2n = sqrt(sum(v2.^2,2)) + eps;
v1u = v1 ./ v1n;
v2u = v2 ./ v2n;

% angle between v1 and v2
cosTheta = sum(v1u .* v2u, 2);
cosTheta = max(min(cosTheta, 1), -1);   % clamp for safety
turnAngle = acos(cosTheta);             % radians

% --- assign hull points to quadrants around centroid ---
cHull = mean(pts, 1);
dx = pts(:,1) - cHull(1);
dy = pts(:,2) - cHull(2);

isTL = (dy < 0) & (dx < 0);
isTR = (dy < 0) & (dx >= 0);
isBR = (dy >= 0) & (dx >= 0);
isBL = (dy >= 0) & (dx < 0);

quadMasks = {isTL, isTR, isBR, isBL};
cornerIdx = zeros(4,1);

for q = 1:4
    mask = quadMasks{q};
    if any(mask)
        [~, localMaxIdx] = max(turnAngle(mask));
        idxInHull = find(mask);
        cornerIdx(q) = idxInHull(localMaxIdx);
    end
end

% remove any empty slots and ensure we have 4 indices
cornerIdx = cornerIdx(cornerIdx > 0);
cornerIdx = unique(cornerIdx, 'stable');

if numel(cornerIdx) < 4
    % fallback: fill remaining with global maxima
    [~, sortedIdx] = sort(turnAngle, 'descend');
    for k = 1:M
        if numel(cornerIdx) >= 4, break; end
        if ~ismember(sortedIdx(k), cornerIdx)
            cornerIdx(end+1) = sortedIdx(k); %#ok<AGROW>
        end
    end
elseif numel(cornerIdx) > 4
    % if weird case, keep the 4 with largest angles
    [~, orderAngle] = sort(turnAngle(cornerIdx), 'descend');
    cornerIdx = cornerIdx(orderAngle(1:4));
end

quadPts = pts(cornerIdx, :);    % 4x2 unordered corners from hull


% --- 6. Order as TL, TR, BR, BL ---
quadPts = order_quad_points(quadPts);

% --- 7. Homography from poster corners to quadPts ---
[H, ~] = build_homography_from_poster(posterRGB, quadPts);

% --- 8. Debug outputs ---
debug.edge          = edgeMap;
debug.candidateMask = candidateMask;
debug.hull          = hull;
debug.quadPts       = quadPts;
debug.cornerIdx     = cornerIdx;   % indices into hull
debug.turnAngle     = turnAngle;
end
