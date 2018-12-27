function sv_figure(index)
    % Fast save specific figure
    bas_dir = '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/KITTI_3D_scene_reconstruction/inter_results/instance_level_seg/';
    fig1 = figure(1);
    F = getframe(fig1);
    [X, ~] = frame2im(F);
    imwrite(X, [bas_dir num2str(index) '_1.png'])
    fig2 = figure(2);
    F = getframe(fig2);
    [X, ~] = frame2im(F);
    imwrite(X, [bas_dir num2str(index) '_2.png'])
    fig3 = figure(3);
    F = getframe(fig3);
    [X, ~] = frame2im(F);
    imwrite(X, [bas_dir num2str(index) '_3.png'])
    % saveas(fig1, [bas_dir num2str(ind) '_1.png'])
end