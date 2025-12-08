function ordered = order_quad_points(pts)
% pts: 4x2 [x y]

if size(pts,1) ~= 4
    error('order_quad_points expects exactly 4 points.');
end

% Compute centroid
cx = mean(pts(:,1));
cy = mean(pts(:,2));

% Compute angle of each point w.r.t. centroid
angles = atan2(pts(:,2)-cy, pts(:,1)-cx);

% Sort by angle (counter-clockwise)
[~, idx] = sort(angles);

ptsSorted = pts(idx,:);

% After CCW sort, ensure we start at top-left (min x+y)
[~, startIdx] = min(sum(ptsSorted,2));
ordered = circshift(ptsSorted, -(startIdx-1), 1);

% Final order: TL, TR, BR, BL (approx)
% (Because of CCW order starting from TL)
end