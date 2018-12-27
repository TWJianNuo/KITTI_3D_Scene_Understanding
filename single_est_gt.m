% Experiments on car, bicycle and pedestrian
txt_path = '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/KITTI_3D_scene_reconstruction/Code/mapping/train_mapping.txt';
contents = read_lines(txt_path); tot_num = length(contents);
used_frame_num = 3; % The frame number being used in testing env, some frame will be skipped due to lack of correpsonding 3d velodine points in KITTI
for i = 1 : used_frame_num
    str_comp = strsplit(contents{i});
    if ~isempty(str_comp{1})
        base_dir = ['/home/ray/Downloads/KITTI_Raw/' str_comp{1} '/' str_comp{2}];
        calib_dir = ['/home/ray/Downloads/KITTI_Raw/' str_comp{1}];
        frame = str2double(str_comp{3});
        [velo_3d, intrinsic, extrinsic] = read_velo(base_dir, calib_dir, frame);
        [rgb, instance_map, semantic_map, demo_img] = read_rgb_ins(i);
        
        %{
        gplane_param = gplane_est(velo_3d);
        velo_3d(:,3) = velo_3d(:,3) - gplane_param(4);
        
        extrinsic(3,4) = extrinsic(3,4) + gplane_param(4);
        P_velo_to_img = intrinsic * extrinsic;
        cubic_inti_guess_debug(semantic_map, P_velo_to_img, velo_3d, rgb);
        visualize(semantic_map, P_velo_to_img, velo_3d, rgb);
        get_instrance(instance_map, P_velo_to_img, velo_3d, rgb);
        %}
        
        % Test codes:
        verify_ground_plane_estimation(velo_3d, intrinsic, extrinsic)
    end
end
%% Ground Plane Estimation and verification
function draw_ground_plane(figure_ind, params)
    xl = xlim; yl = ylim; 
    pts = [xl(1) yl(1); xl(1) yl(2); xl(2) yl(2); xl(2) yl(1)];
    zz = (-params(1) * pts(:,1) - params(2) * pts(:,2) - params(4)) / params(3);
    pts = [pts zz];
    figure(figure_ind); hold on; patch('Faces',[1 2 3 4],'Vertices',pts,'FaceAlpha', 0.5)
end
function verify_ground_plane_estimation(velo_3d, intrinsic, extrinsic)
    % Verify the ground plane alignment
    
    T = intrinsic * extrinsic;
    [~,~,selector] = project(velo_3d, T, true);
    velo_3d = velo_3d(selector, :); % Filter out points not within the plane
    
    [affine_matrx, flat_velo_3d, gplane_param] = gplane_align(velo_3d, intrinsic, extrinsic);
    
    pts2d1 = project(velo_3d, T);
    pts2d2 = project(flat_velo_3d, T * inv(affine_matrx));
    diff = sum(sum(abs(pts2d1 - pts2d2)));
    fprintf('summed value differences between before and after projection points is %2d\n', diff)
    
    figure(1); clf; scatter3(velo_3d(:,1),velo_3d(:,2),velo_3d(:,3),3,'r','fill'); axis equal;
    draw_ground_plane(1, gplane_param);
    figure(2); clf; scatter3(flat_velo_3d(:,1),flat_velo_3d(:,2),flat_velo_3d(:,3),3,'g','fill'); axis equal;
end
%% Others
function [tot_linear_ind] = cubic_lines_of_2d(l, w, cubic, intrinsic_params, extrinsic_params, label, linear_ind_org, depth_map)
    tot_linear_ind = zeros(0); building_th = 3;
    sky_ind = 1; Road_ind = 3; Sidewalk_ind = 4; Lanemarking_ind = 12; 
    permitted_ind = [sky_ind Road_ind Sidewalk_ind Lanemarking_ind];
    pts3d = zeros(8,4);
    try
        mh = cubic{1}.mh;
        if mh > building_th
            mh = 0;
        end
    catch
        mh = 0;
    end
    
    % Deal with when camera inside the cubic shape
    cam_origin = (inv(extrinsic_params) * [0 0 0 1]')';
    theta = cubic{1}.theta; center = mean(cubic{5}.pts); xc = center(1); yc = center(2);
    cub_l = cubic{1}.length1; cub_w = cubic{2}.length1;
    aff = [[1 0 1]', [0 1 1]', [0 0 1]'] * inv([[xc + cos(theta), yc + sin(theta), 1]', [xc - sin(theta), yc + cos(theta), 1]', [xc, yc, 1]']);
    cam_origin_ = (aff * [cam_origin(:,1:2) 1]')';
    is_in = (abs(cam_origin_(:,1)) < cub_l / 2) & (abs(cam_origin_(:,2)) < cub_w / 2);
    if is_in
        tot_linear_ind = zeros(0);
        return;
        %{
        corner_pt = [cub_l / 2, -cub_l / 2, cub_w / 2, -cub_w / 2];
        dis_vals = [abs(cam_origin_(1) - corner_pt(1:2)), abs(cam_origin_(2) - corner_pt(3:4))];
        ind = find(min(dis_vals)); ind = ind(1); ind = 2
        corner_pt(ind) = sign(corner_pt(ind)) * (abs(corner_pt(ind)) - dis_vals(ind) * ratio);
        cub_l = corner_pt(1) - corner_pt(2); cub_w = corner_pt(3) - corner_pt(4);
        center_pt = [mean(corner_pt(1:2)) mean(corner_pt(3:4)) 1];
        center_pt = (inv(aff) * center_pt')';
        xc = center_pt(1); yc = center_pt(2);
        cubic = generate_cuboid_by_center(xc, yc, theta, cub_l, cub_w, cub_h);
        %}
    end
    % figure(1); clf; draw_cubic_shape_frame(cubic); hold on; scatter3(cam_origin(1), cam_origin(2), cam_origin(3), 15, 'r', 'fill');
    
    
    
    for i = 1 : 4
        pts3d(i, :) = [cubic{i}.pts(1, :) 1];
        pts3d(i, 3) = pts3d(i, 3) + mh;
    end
    for i = 5 : 8
        pts3d(i, :) = [cubic{5}.pts(i - 4, :) 1];
    end
    % Process cubic shape exceeding camera plane
    lines_3d = zeros(12, 8);
    lines_3d(4, :) = [pts3d(4, :) pts3d(1, :)];
    lines_3d(12, :) = [pts3d(5, :) pts3d(8, :)];
    for i = 1 : 3
        lines_3d(i, :) = [pts3d(i, :) pts3d(i+1, :)];
    end
    for i = 1 : 4
        lines_3d(4 + i, :) = [pts3d(i, :), pts3d(i + 4, :)];
    end
    for i = 1 : 3
        lines_3d(8 + i, :) = [pts3d(i + 4, :) pts3d(i + 5, :)];
    end
    %{
    % Verify the drawing
    figure(1); clf; 
    for i = 1 : size(lines_3d, 1)
        tmp = [lines_3d(i, 1:3); lines_3d(i,5:7)];
        hold on; plot3(tmp(:,1), tmp(:,2), tmp(:,3));
    end
    %}
    expand_num = 100;
    tmp = [lines_3d(:,1:4);lines_3d(:,5:8)];
    tmp = (extrinsic_params * tmp')'; selector = reshape(tmp(:,3) < 1e-5, [size(lines_3d,1), 2]);
    for i = 1 : size(selector,1)
        if sum(selector(i,:)) == 1
            ind_bad = find(selector(i,:)); ind_good = find(~selector(i,:));
            ind_bad = (ind_bad - 1) * 4 + 1; ind_good = (ind_good - 1) * 4 + 1;
            pts_ = [
                lines_3d(i, ind_good) lines_3d(i, ind_good + 1) lines_3d(i, ind_good + 2);
                lines_3d(i, ind_bad) lines_3d(i, ind_bad + 1) lines_3d(i, ind_bad + 2);
                ];
            pts_ = [linspace(pts_(1,1), pts_(2,1), expand_num)', linspace(pts_(1,2), pts_(2,2), expand_num)', linspace(pts_(1,3), pts_(2,3), expand_num)', ones(expand_num, 1)];
            pts__ = (extrinsic_params * pts_')'; pos_ind = find(pts__(:,3) > 0); pos_ind = ceil(length(pos_ind) * 0.9);
            lines_3d(i, ind_bad : ind_bad + 3) = pts_(pos_ind, :);
        end
        if sum(selector(i,:)) == 2
            lines_3d(i,:) = ones(1,8) * inf;
        end
    end
    lines_3d = [lines_3d(:,1:4); lines_3d(:, 5:8)];
    
    
    pts2d = (intrinsic_params * extrinsic_params * [pts3d(:, 1:3) ones(size(pts3d,1),1)]')';
    depth = pts2d(:,3); depth_rep = min(depth);
    pts2d(:, 1) = pts2d(:,1) ./ depth; pts2d(:,2) = pts2d(:,2) ./ depth; pts2d = round(pts2d(:,1:2));
    lines = (intrinsic_params * extrinsic_params * lines_3d')';
    lines(:,1) = lines(:,1) ./ lines(:,3); lines(:,2) = lines(:,2) ./ lines(:,3); lines = lines(:, 1:2);
    lines = [lines(1 : 12, :) lines(13 : 24, :)];
    %{
    lines = zeros(12, 4);
    lines(4, :) = [pts2d(4, :) pts2d(1, :)];
    lines(12, :) = [pts2d(5, :) pts2d(8, :)];
    for i = 1 : 3
        lines(i, :) = [pts2d(i, :) pts2d(i+1, :)];
    end
    for i = 1 : 4
        lines(4 + i, :) = [pts2d(i, :), pts2d(i + 4, :)];
    end
    for i = 1 : 3
        lines(8 + i, :) = [pts2d(i + 4, :) pts2d(i + 5, :)];
    end
    %}
    x1 = lines(:,1); y1 = lines(:,2); x2 = lines(:,3); y2 = lines(:,4);
    cross_pts = [...
        ones(12,1), (1 - x1) .* (y2 - y1) ./ (x2 - x1) + y1, ...
        ones(12,1) * l, (y2 - y1) ./ (x2 - x1) .* (l - x1) + y1, ...
        (1 - y1) .* (x2 - x1) ./ (y2 - y1) + x1, ones(12,1), ...
        (x2 - x1) ./ (y2 - y1) .* (w - y1) + x1, ones(12,1) * w ...
        ];

    selector1 = [
        cross_pts(:,1) >=1 & cross_pts(:,1) <= l & cross_pts(:,2) >= 1 & cross_pts(:,2) <= w, ...
        cross_pts(:,3) >=1 & cross_pts(:,3) <= l & cross_pts(:,4) >= 1 & cross_pts(:,4) <= w, ...
        cross_pts(:,5) >=1 & cross_pts(:,5) <= l & cross_pts(:,6) >= 1 & cross_pts(:,6) <= w, ...
        cross_pts(:,7) >=1 & cross_pts(:,7) <= l & cross_pts(:,8) >= 1 & cross_pts(:,8) <= w, ...
        ];
    selector2 = [
        lines(:,1) >= 1 & lines(:,1) <= l & lines(:,2) >= 1 & lines(:,2) <= w, ...
        lines(:,3) >= 1 & lines(:,3) <= l & lines(:,4) >= 1 & lines(:,4) <= w, ...
        ];
    selector2 = ~selector2; valid_lines = true(12,1);
    for i = 1 : 12
        if selector2(i,1) && selector2(i,2)
            valid_lines(i) = false;
            continue
        end
        if sum(selector1(i,:)) == 1
            ind1 = find(selector1(i,:));
            ind2 = find(selector2(i,:));
            lines(i, 2*ind2 - 1 : 2*ind2) = [cross_pts(i, 2*ind1 - 1), cross_pts(i, 2*ind1)];
            continue;
        end
        for j = 1 : 2
            if selector2(i,j)
                ind = find(selector1(i,:));
                if isempty(ind)
                    valid_lines(i) = false;
                    continue
                end
                pts_out = [lines(i, 2*j - 1), lines(i, 2 * j)];
                pts1 = [cross_pts(i, 2*ind(1) - 1), cross_pts(i, 2*ind(1))];
                pts2 = [cross_pts(i, 2*ind(2) - 1), cross_pts(i, 2*ind(2))];
                d1 = norm(pts_out - pts1); d2 = norm(pts_out - pts2);
                if d1 < d2
                    lines(i, 2*j - 1 : 2*j) = pts1;
                else
                    lines(i, 2*j - 1 : 2*j) = pts2;
                end
            end
        end
    end
    lines = lines(valid_lines,:);
    % figure(1); clf;
    % for i = 1 : size(lines,1)
    %     hold on; plot([lines(i,1); lines(i,3)],[lines(i,2) lines(i,4)])
    % end
    bmap = false(w, l); bmap(linear_ind_org) = true; se = strel('square',1); linear_ind_org = find(imdilate(bmap,se));
    dotted_line_inter_ind = 4; min_x = inf; min_y = inf;
    for i = 1 : size(lines,1)
        leng1 = abs(lines(i,1) - lines(i,3));
        leng2 = abs(lines(i,2) - lines(i,4));
        if leng1 > leng2
            leng = leng1;
        else
            leng = leng2;
        end
        xx = round(linspace(lines(i,1), lines(i,3), leng));
        yy = round(linspace(lines(i,2), lines(i,4), leng));
        linear_ind = sub2ind([w, l], yy, xx);
        selector = ismember(label(linear_ind), permitted_ind) | ismember(linear_ind, linear_ind_org) | depth_map(linear_ind) > depth_rep;
        % Generate dotted lines
        dotted_selector = selector(~selector);
        for j = 1 : dotted_line_inter_ind
            if j <= dotted_line_inter_ind / 2
                dotted_selector(j : dotted_line_inter_ind : end) = true;
            end
        end
        selector(~selector) = dotted_selector;
        tot_linear_ind = [tot_linear_ind linear_ind(selector)];
        if min_x > min(xx(selector))
            min_x = min(xx(selector));
        end
        if min_y > min(yy(selector))
            min_y = min(yy(selector));
        end
    end 
    %{
    % lines(xx2, yy2 * 2 - 1) = cross_pts(xx1, yy1 * 2 - 1);
    % lines(xx2, yy2 * 2) = cross_pts(xx1, yy1 * 2);
    for i = 9 : 9
        figure(1); clf; scatter(lines(i,1),lines(i,2),10,'r','fill');
        hold on; scatter(lines(i,3),lines(i,4),10,'r','fill');
        hold on; scatter(cross_pts(i,1),cross_pts(i,2),10,'b','fill');
        hold on; scatter(cross_pts(i,3),cross_pts(i,4),10,'b','fill');
        hold on; scatter(cross_pts(i,5),cross_pts(i,6),10,'b','fill');
        hold on; scatter(cross_pts(i,7),cross_pts(i,8),10,'b','fill');
        axis equal
    end
    selector = [
        ((cross_pts(:, 1) - x1) .* (cross_pts(:, 1) - x2) < 0) ...
        & ((cross_pts(:, 2) - y1) .* (cross_pts(:, 2) - y2) < 0), ...
        ((cross_pts(:, 3) - x1) .* (cross_pts(:, 3) - x2) < 0) ...
        & ((cross_pts(:, 4) - y1) .* (cross_pts(:, 4) - y2) < 0), ...
        ((cross_pts(:, 5) - x1) .* (cross_pts(:, 5) - x2) < 0) ...
        & ((cross_pts(:, 6) - y1) .* (cross_pts(:, 6) - y2) < 0), ...
        ((cross_pts(:, 7) - x1) .* (cross_pts(:, 7) - x2) < 0) ...
        & ((cross_pts(:, 8) - y1) .* (cross_pts(:, 8) - y2) < 0), ...
        ];
    [xx, yy] = find(selector);
    if ~isempty(xx)
        selector1 = x1 < 0 | x1 > l | y1 < 0 | y1 > w;
        selector2 = x2 < 0 | x2 > l | y2 < 0 | y2 > w;
        lines(selector1, 1) = cross_pts(selector1, yy * 2 - 1);
        lines(selector1, 2) = cross_pts(selector1, yy * 2);
        lines(selector2, 1) = cross_pts(selector2, yy * 2 - 1);
        lines(selector2, 1) = cross_pts(selector2, yy * 2 - 1);
    end
    if sum(selector) > 0
        selector_ = [
            ((cross_pts(selector, 1) - x1(selector)) .* (cross_pts(selector, 1) .* x2(selector)) < 0) ...
            & ((cross_pts(selector, 2) - y1(selector)) .* (cross_pts(selector, 2) .* y2(selector)) < 0), ...
            ((cross_pts(selector, 3) - x1(selector)) .* (cross_pts(selector, 3) .* x2(selector)) < 0) ...
            & ((cross_pts(selector, 4) - y1(selector)) .* (cross_pts(selector, 4) .* y2(selector)) < 0), ...
            ((cross_pts(selector, 5) - x1(selector)) .* (cross_pts(selector, 5) .* x2(selector)) < 0) ...
            & ((cross_pts(selector, 6) - y1(selector)) .* (cross_pts(selector, 6) .* y2(selector)) < 0), ...
            ((cross_pts(selector, 7) - x1(selector)) .* (cross_pts(selector, 8) .* x2(selector)) < 0) ...
            & ((cross_pts(selector, 7) - y1(selector)) .* (cross_pts(selector, 8) .* y2(selector)) < 0), ...
            ];
        
        [~, yy] = find(selector_, 2);
    end
    %}
    %{
    for i = 1 : 12
        img = step(shapeInserter, img, int32([lines(i, 1) lines(i, 2) lines(i, 3) lines(i, 4)]));
    end
    %}
end
function cubic_inti_guess_debug(semantic_map, P_velo_to_img, velo_3d, rgb)
    maxdist = 2; minClusterSize = 20; method = 'point'; mergeflag = 'merge';
    [cat_str, cat_id] = acquire_label();
    valid_entry_name = {
        'vegetation';
        };
    valid_entry = false(length(cat_id),1);
    for i = 1 : length(valid_entry)
        valid_entry(i) = ismember(cat_str{i}, valid_entry_name);
    end
    valid_entry = cat_id(valid_entry);
    
    [pts_2d, ~, selector_] = project(velo_3d,P_velo_to_img,true);
    velo_3d = velo_3d(selector_,:);
    pts_2d = round(pts_2d(:,1:2)); linaer_2d = sub2ind(size(semantic_map), pts_2d(:,2), pts_2d(:,1));
    selector = ismember(semantic_map(linaer_2d), valid_entry);
    
    refined_tran_desp = cal_triang(velo_3d(selector, 1:3)); indice = unique(refined_tran_desp(:));
    selector_maks = false(sum(selector),1); selector_maks(indice) = true; selector(selector) = selector_maks;
    clustersXY = clusterXYpoints(velo_3d(selector,1:3),maxdist,minClusterSize,method,mergeflag);
    
    velo_3d_tmp = velo_3d(selector,:); cubics = cell(length(clustersXY),1);
    for i = 1 : length(clustersXY)
        index = clustersXY{i};
        [~, cubics{i}] = estimate_rectangular(velo_3d_tmp(index,:));
    end
    figure(1); clf; 
    for i = 1 : length(clustersXY)
        color = rand(1,3); index = clustersXY{i};
        scatter3(velo_3d_tmp(index,1),velo_3d_tmp(index,2),velo_3d_tmp(index,3),3,color,'fill'); hold on
        draw_cubic_shape_frame(cubics{i}); hold on
    end
    axis equal
    
    color = zeros(sum(selector), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d(selector)); color(:,2) = g(linaer_2d(selector)); color(:,3) = b(linaer_2d(selector)); color = color / 255;
    figure(2); clf; scatter3(velo_3d(selector,1), velo_3d(selector,2), velo_3d(selector,3), 3, color, 'fill'); axis equal
    % r_ = r; g_ = g; b_ = b;
    % r_(linaer_2d(selector)) = 255; g_(linaer_2d(selector)) = 0; b_(linaer_2d(selector)) = 0;
    % recons = cat(3, r_, g_, b_);
    figure(3); clf; imshow(rgb); hold on; scatter(pts_2d(selector,1),pts_2d(selector,2),6,'g','fill')
    
end
function cubic_inti_guess(semantic_map, P_velo_to_img, velo_3d)
end
function refined_tran_desp = cal_triang(pts_3d)
    area_th = 1; 
    l_th = 1;
    raw_tran_desp = compute_delaunay(pts_3d)';
    [area, AB_nrom, AC_norm, BC_norm] = triang_size(pts_3d, raw_tran_desp);
    selector = (AB_nrom < l_th) & (AC_norm < l_th) & (BC_norm < l_th);
    refined_tran_desp = raw_tran_desp(selector, :);
    figure(1),clf,plot_mesh(pts_3d, refined_tran_desp)
end
function [area, AB_nrom, AC_norm, BC_norm] = triang_size(pts_3d, tran_desp)
    AB = pts_3d(tran_desp(:,2),:) - pts_3d(tran_desp(:,1),:);
    AC = pts_3d(tran_desp(:,3),:) - pts_3d(tran_desp(:,1),:);
    BC = pts_3d(tran_desp(:,3),:) - pts_3d(tran_desp(:,2),:);
    AB_nrom = vecnorm(AB,2,2);
    AC_norm = vecnorm(AC,2,2);
    BC_norm = vecnorm(BC,2,2);
    dot_re = dot(AB', AC')';
    area = sqrt((AB_nrom .* AC_norm).^2 - (dot_re).^2) / 2;
end
function get_instrance(instance_map, P_velo_to_img, velo_3d, rgb)
    unique_ins = unique(instance_map); 
    unique_ins= unique_ins(unique_ins~=0);
    
    [pts_2d, ~, selector_] = project(velo_3d,P_velo_to_img,true);
    velo_3d = velo_3d(selector_,:); % Only preserve velo points within the image
    pts_2d = round(pts_2d(:,1:2)); linaer_2d = sub2ind(size(instance_map), pts_2d(:,2), pts_2d(:,1));
    car_selector = ismember(instance_map(linaer_2d), unique_ins);
    car_pts3d = velo_3d(car_selector,:); car_pts2d = pts_2d(car_selector,:);
    
    r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    r_ = r; g_ = g; b_ = b;
    r_(linaer_2d(car_selector)) = 255; g_(linaer_2d(car_selector)) = 0; b_(linaer_2d(car_selector)) = 0;
    recons = cat(3, r_, g_, b_);
    figure(2); clf; imshow(recons); hold on; scatter(car_pts2d(:,1),car_pts2d(:,2),3,'g','fill')
    for i = 1 : length(unique_ins)
        sub_selector = (instance_map(linaer_2d(car_selector)) == unique_ins(i)); % faster
        tmp_pts3d = car_pts3d(sub_selector,:); tmp_pts2d = car_pts2d(sub_selector,:);
        visualize_velo(rgb, tmp_pts3d, P_velo_to_img);
        figure(2); clf; imshow(rgb);
        hold on; scatter(tmp_pts2d(:,1),tmp_pts2d(:,2),3,'r','fill')
    end
    %{
    figure(1); clf; imshow(rgb);
    hold on; scatter(pts_2d(:,1),pts_2d(:,2),3,'r','fill')
    figure(2); clf; scatter3(velo_3d(:,1),velo_3d(:,2),velo_3d(:,3),3,'r','fill'); axis equal
    
    
    color = zeros(sum(car_selector), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d(car_selector)); color(:,2) = g(linaer_2d(car_selector)); color(:,3) = b(linaer_2d(car_selector)); color = color / 255;
    figure(1); clf; scatter3(velo_3d(car_selector,1), velo_3d(car_selector,2), velo_3d(car_selector,3), 3, color, 'fill'); axis equal
    r_ = r; g_ = g; b_ = b;
    r_(linaer_2d(car_selector)) = 255; g_(linaer_2d(car_selector)) = 0; b_(linaer_2d(car_selector)) = 0;
    recons = cat(3, r_, g_, b_);
    figure(2); clf; imshow(recons)
    % visualize_velo(rgb, velo_3d(selector_,:), P_velo_to_img)
    %}
end
function contents = read_lines(txt_path)
    % Seg text file according to '\n'
    pre_alloc_num = 200;
    fid = fopen(txt_path);
    tline = fgetl(fid);
    contents = cell(pre_alloc_num,1);
    count = 1;
    while ischar(tline)
        contents{count} = tline;
        tline = fgetl(fid);
        count = count + 1;
    end
    fclose(fid);
end
function visualize(semantic_map, P_velo_to_img, velo_3d, rgb)
    [cat_str, cat_id] = acquire_label();
    valid_entry_name = {
        'vegetation';
        };
    valid_entry = false(length(cat_id),1);
    for i = 1 : length(valid_entry)
        valid_entry(i) = ismember(cat_str{i}, valid_entry_name);
    end
    valid_entry = cat_id(valid_entry);
    
    [pts_2d, depth, selector_] = project(velo_3d,P_velo_to_img,true);
    velo_3d = velo_3d(selector_,:);
    pts_2d = round(pts_2d(:,1:2)); linaer_2d = sub2ind(size(semantic_map), pts_2d(:,2), pts_2d(:,1));
    selector = ismember(semantic_map(linaer_2d), valid_entry);
    
    
    color = zeros(sum(selector), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d(selector)); color(:,2) = g(linaer_2d(selector)); color(:,3) = b(linaer_2d(selector)); color = color / 255;
    figure(1); clf; scatter3(velo_3d(selector,1), velo_3d(selector,2), velo_3d(selector,3), 3, color, 'fill'); axis equal
    r_ = r; g_ = g; b_ = b;
    r_(linaer_2d(selector)) = 255; g_(linaer_2d(selector)) = 0; b_(linaer_2d(selector)) = 0;
    recons = cat(3, r_, g_, b_);
    figure(2); clf; imshow(recons)
    % visualize_velo(rgb, velo_3d(selector,:), P_velo_to_img)
end
function [rgb, instance_map, semantic_map, demo_img] = read_rgb_ins(ind)
    path = '/home/ray/Downloads/KITTI_Raw/sementics/data_semantics/training/';
    img_path = 'image_2/';
    ins_path = 'instance/';
    % seman_path = 'semantic/';
    % semantic_map = imread([path seman_path num2str(ind-1,'%06d') '_10.png']);
    seman_rgb_path = 'semantic_rgb/';
    rgb = imread([path img_path num2str(ind-1,'%06d') '_10.png']);
    instance_map_ = imread([path ins_path num2str(ind-1,'%06d') '_10.png']);
    demo_img = imread([path seman_rgb_path num2str(ind-1,'%06d') '_10.png']);
    instance_map = mod(instance_map_, 256);
    semantic_map = instance_map_ / 256;
end
function [cat_str, cat_id] = acquire_label()
    cat_str = {
        'unlabeled';
        'ego vehicle';
        'rectification border';
        'out of roi';
        'static';
        'dynamic';
        'ground';
        'road';
        'sidewalk';
        'parking';
        'rail track';
        'building';
        'wall';
        'fence';
        'guard rail';
        'bridge';
        'tunnel';
        'pole';
        'polegroup';
        'traffic light';
        'traffic sign';
        'vegetation';
        'terrain';
        'sky';
        'person';
        'rider';
        'car';
        'truck';
        'bus';
        'caravan';
        'trailer';
        'train';
        'motorcycle';
        'bicycle';
        'license plate';
        };
    cat_id = [
        0;
        1;
        2;
        3;
        4;
        5;
        6;
        7;
        8;
        9;
        10;
        11;
        12;
        13;
        14;
        15;
        16;
        17;
        18;
        19;
        20;
        21;
        22;
        23;
        24;
        25;
        26;
        27;
        28;
        29;
        30;
        31;
        32;
        33;
        -1;
        ];
end
function [p_out, depth, selector] = project(p_in,T,is_in_img,img_size)
    
    % dimension of data and projection matrix
    dim_norm = size(T,1);
    dim_proj = size(T,2);
    
    % do transformation in homogenuous coordinates
    p2_in = p_in;
    if size(p2_in,2)<dim_proj
        p2_in(:,dim_proj) = 1;
    end
    p2_out = (T*p2_in')';
    depth = p2_out(:,3);
    % normalize homogeneous coordinates:
    p_out = [p2_out(:,1) ./ depth, p2_out(:,2) ./ depth];
    
    if nargin > 2
        if is_in_img
            if nargin >= 3 && nargin < 4
                image_size = [376, 1241];
            elseif nargin == 4
                image_size = img_size;
            end
            p_outd = round(p_out);
            selector = (p_outd(:,1) > 0) & (p_outd(:,2) > 0) & (p_outd(:,1) < image_size(2)) & (p_outd(:,2) < image_size(1));
            p_out = p_out(selector,:);
            depth = depth(selector);
        end
    end
    
end
function visualize_velo(rgb, velo, T)
    [p_out, depth, selector] = project(velo,T,true);
    p_out = round(p_out); image_size = [376, 1241];
    linaer_2d = sub2ind(image_size, p_out(:,2), p_out(:,1));
    color = zeros(length(depth), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d); color(:,2) = g(linaer_2d); color(:,3) = b(linaer_2d); color = color / 255;
    figure(1); clf; scatter3(velo(selector,1), velo(selector,2), velo(selector,3), 3, color, 'fill'); axis equal;
end