function est_perfec_inst_label()
    dbstop error;
    txt_path = '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/KITTI_3D_scene_reconstruction/Code/mapping/train_mapping.txt';
    fid = fopen(txt_path);
    tline = fgetl(fid);
    count = 1;
    while ischar(tline)
        tline = fgetl(fid);
        count = count + 1;
        if ~isempty(tline)
            [velo_3d, P_velo_to_img] = read_spec_velodine(tline);
            [rgb, instance_map, semantic_map, demo_img] = read_rgb_ins(count);
            visualize(semantic_map, P_velo_to_img, velo_3d, rgb);
        end
    end
    fclose(fid);
    disp(count)
end
function visualize(semantic_map, P_velo_to_img, velo_3d, rgb)
    [cat_str, cat_id] = acquire_label();
    valid_entry_name = {
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
        };
    valid_entry = false(length(cat_id),1);
    for i = 1 : length(valid_entry)
        valid_entry(i) = ismember(cat_str{i}, valid_entry_name);
    end
    valid_entry = cat_id(valid_entry);
    
    pts_2d = (P_velo_to_img * velo_3d')'; 
    pts_2d(:,1) = pts_2d(:,1) ./ pts_2d(:,3);
    pts_2d(:,2) = pts_2d(:,2) ./ pts_2d(:,3);
    depth = pts_2d(:,3);
    pts_2d = round(pts_2d(:,1:2)); linaer_2d = sub2ind(size(semantic_map), pts_2d(:,2), pts_2d(:,1));
    selector = ismember(semantic_map(linaer_2d), valid_entry); 
    
    color = zeros(sum(selector), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d(selector)); color(:,2) = g(linaer_2d(selector)); color(:,3) = b(linaer_2d(selector)); color = color / 255;
    figure(1); clf; scatter3(velo_3d(selector,1), velo_3d(selector,2), velo_3d(selector,3), 3, color, 'fill'); axis equal
    r_ = r; g_ = g; b_ = b;
    r_(linaer_2d(selector)) = 255; g_(linaer_2d(selector)) = 0; b_(linaer_2d(selector)) = 0;
    recons = cat(3, r_, g_, b_);
    figure(2); clf; imshow(recons)
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
function [velo_3d, P_velo_to_img] = read_spec_velodine(map_entry)
    str_comp = strsplit(map_entry);
    base_dir = '/home/ray/Downloads/KITTI_Raw/';
    [velo_3d, P_velo_to_img] = read_velo([base_dir str_comp{1} '/' str_comp{2}], [base_dir str_comp{1}], str2double(str_comp{3}));
end