function [plane_param, inlierIndices] = gplane_est(pts_3d)
    maxDistance = 0.02; pts_3d_ = pts_3d(:,1:3);
    ptCloud = pointCloud(pts_3d_);
    maxAngularDistance = 5;
    referenceVector = [0 0 1];
    [model1,inlierIndices,~] = pcfitplane(ptCloud,maxDistance,referenceVector,maxAngularDistance);
    plane_param = model1.Parameters;
end