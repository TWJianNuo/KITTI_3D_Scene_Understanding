function visualize_velo2(rgb1, velo1, new_velo1, T1, rgb2, velo2, T2, is_save)
    % This is a specific visualize for frame alignment only
    [~, color1, selector1] = get_color(rgb1, velo1, T1);
    [velo2, color2, selector2] = get_color(rgb2, velo2, T2);
    figure(1); clf; scatter3(new_velo1(selector1,1), new_velo1(selector1,2), new_velo1(selector1,3), 3, color1, 'fill'); 
    hold on; scatter3(velo2(selector2,1), velo2(selector2,2), velo2(selector2,3), 3, color2, 'fill'); axis equal;
    figure(2); clf; scatter3(new_velo1(selector1,1), new_velo1(selector1,2), new_velo1(selector1,3), 3, 'r', 'fill'); 
    hold on; scatter3(velo2(selector2,1), velo2(selector2,2), velo2(selector2,3), 3, 'g', 'fill'); axis equal;
    if is_save
        path = '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/KITTI_3D_scene_reconstruction/inter_results/Frame_alignment_re/'; 
        n = numel(dir([path '*.png'])); order = ceil(n / 3) + 1;
        figure(1); F = getframe(gcf); [X, ~] = frame2im(F); imwrite(X, [path num2str(order) '_1.png']); 
        figure(2); F = getframe(gcf); [X, ~] = frame2im(F); imwrite(X, [path num2str(order) '_2.png']);
        imwrite(rgb2, [path num2str(order) '_3.png'])
    end
end