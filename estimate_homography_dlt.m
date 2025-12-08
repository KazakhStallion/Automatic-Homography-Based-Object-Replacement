function H = estimate_homography_dlt(srcPts, dstPts)
% srcPts, dstPts: Nx2, at least N=4
n = size(srcPts,1);
if n < 4
    error('Need at least 4 point correspondences for DLT.');
end

A = zeros(2*n, 9);
for i = 1:n
    x = srcPts(i,1);  y = srcPts(i,2);
    X = dstPts(i,1);  Y = dstPts(i,2);
    A(2*i-1,:) = [-x -y -1  0  0  0  x*X  y*X  X];
    A(2*i  ,:) = [ 0  0  0 -x -y -1  x*Y  y*Y  Y];
end

[~,~,V] = svd(A);
h = V(:,end);
H = reshape(h,3,3)';    % note transpose
H = H ./ H(3,3);
end