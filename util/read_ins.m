function [instance_map, semantic_map] = read_ins(base_dir, frame)
    % Specific for the sementics data
    cam = 2;
    path = sprintf('%s/image_%02d/data/%010d.png', base_dir, cam, frame);
    rgb = imread(path);
    path = '/home/ray/Downloads/KITTI_Raw/sementics/data_semantics/training/';
    img_path = 'image_2/';
    ins_path = 'instance/';
    seman_rgb_path = 'semantic_rgb/';
    rgb = imread([path img_path num2str(ind-1,'%06d') '_10.png']);
    instance_map_ = imread([path ins_path num2str(ind-1,'%06d') '_10.png']);
    demo_img = imread([path seman_rgb_path num2str(ind-1,'%06d') '_10.png']);
    instance_map = mod(instance_map_, 256);
    semantic_map = instance_map_ / 256;
end