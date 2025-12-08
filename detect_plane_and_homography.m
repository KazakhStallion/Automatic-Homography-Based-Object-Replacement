function [H, quadPts, debug] = detect_plane_and_homography(sceneRGB, posterRGB)

sceneGray = rgb2gray(sceneRGB);

% Canny edges
edgeMap = edge(sceneGray, 'canny');


%%%%% WORK on the stuf below%%%%%%
% Find all contours
B = bwboundaries(edgeMap);

bestArea = -inf;
bestQuad = [];

for k = 1:length(B)
    contour = B{k};              % Nx2 [row, col]
    pts = fliplr(contour);       % convert to [x, y]

    % Polygon simplify
    epsilon = 5;                 % adjust 1–10 depending on image scale
    approx = reducepoly(pts, epsilon);

    if size(approx,1) == 4       % a quadrilateral
        % Compute polygon area
        area = polyarea(approx(:,1), approx(:,2));
        if area > bestArea
            bestArea = area;
            bestQuad = approx;
        end
    end
end

if isempty(bestQuad)
    error('No 4-corner polygon found.');
end

quadPts = bestQuad;

% Order TL TR BR BL
quadPts = order_quad_points(quadPts);

%%% UNtil HERE %%%%

% Compute H
[H, ~] = build_homography_from_poster(posterRGB, quadPts);

debug.edge      = edgeMap;
debug.quadPts   = quadPts;
debug.contours  = B;

end
