% Title: VLfeat provided SIFT feature detector and descriptor
% Author: VLfeat, used by Shreyash Annapureddy
% Date: 18/02/2016
% Updated: 7/03/2015
%% SIFT detector and descriptor
function [keypoints,descriptor] = SIFT(img)
[rows,cols,dimen] = size(img);
if dimen > 1
    I = single(rgb2gray(img));
else
    I = single(img);
end
% The matrix "keypoints" has a column for each frame, A frame is defined
% as a:
%       1. disk of center keypoints(1:2)
%       2. scale keypoints(3)
%       3. orientation keypoints(4)

edge_thresh = 0; % Threshold that eliminates peaks of DoG scale space whose curvature is too small
peak_thresh = 0; % Threshold that filters peaks of DoG scale space that are too small (abs vals)


[keypoints, descriptor] = vl_sift(I);
% Randomly select n number of features
% perm = randperm(size(keypoints,2));
% [~,numOfFeatures] = size(perm);
% sel  = perm(1:numOfFeatures*0.7);

% keypoints  = keypoints(:,sel);
% descriptor = descriptor(:,sel);

end

