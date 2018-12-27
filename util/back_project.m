function recons_pt = back_project(pixel_loc, depth, T)
    expanded_pts = [pixel_loc(:,1).*depth, pixel_loc(:,2).*depth, depth, ones(length(depth),1)];
    T(4,4) = 1;
    recons_pt = (inv(T) * expanded_pts')';
end