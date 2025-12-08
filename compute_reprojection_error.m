function err = compute_reprojection_error(H, srcPts, dstPts)
% srcPts: Nx2 in poster
% dstPts: Nx2 in scene
n = size(srcPts,1);
srcH = [srcPts, ones(n,1)]';    % 3xN

proj = H * srcH;
proj = proj ./ proj(3,:);       % normalize

projPts = proj(1:2,:)';         % Nx2
diffs   = projPts - dstPts;
errVec  = sqrt(sum(diffs.^2,2));
err     = mean(errVec);
end