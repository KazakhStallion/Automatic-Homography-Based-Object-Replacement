function [H, srcPts] = build_homography_from_poster(posterRGB, quadPts)
[Himg, Wimg, ~] = size(posterRGB);

% Poster coordinates (pixel corners)
srcPts = [ 1      1;
           Wimg   1;
           Wimg   Himg;
           1      Himg ];
dstPts = quadPts;  % detected in scene

H = estimate_homography_dlt(srcPts, dstPts);
end