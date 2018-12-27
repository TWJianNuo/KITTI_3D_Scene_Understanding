function rgb_mask = get_mask()
    % In case 3D velodine points are sparse, using a mask to filter out 2d
    % image positions where a 3D point unavailable
    path = 'rgbmask.mat';
    if exist(path)
        load(path);
    else
        cam = 2; % 0-based-index
        calib_dir = '/home/ray/Downloads/KITTI_Raw/2011_09_26/';
        base_dir = '/home/ray/Downloads/KITTI_Raw/2011_09_26/2011_09_26_drive_0001_sync/';
        img_path = sprintf('%s/image_%02d/data/', base_dir, cam);
        
        img_path_set = dir(img_path); img_size = look_up_specification('img_size'); rgb_mask = zeros(img_size);
        for i = 1 : size(img_path_set,1) - 2
            frame = i - 1;
            fprintf('Current frame is: %03d frame.\n',i)
            [velo, p_matrix] = read_velo(base_dir, calib_dir, frame);
            % rgb_img = read_rgb(base_dir, frame);
            p_out = project(velo,p_matrix); p_out = round(p_out);
            selector = (p_out(:,1) > 0) & (p_out(:,1) < img_size(2)) & (p_out(:,2) > 0) & (p_out(:,2) < img_size(1));
            p_out = p_out(selector,:); linear_ind = sub2ind(img_size, p_out(:,2), p_out(:,1)); 
            rgb_mask(linear_ind) = rgb_mask(linear_ind) + 1;
        end
        rgb_mask = rgb_mask > mean(mean(rgb_mask)) / 5;
        save(path, 'rgb_mask')
    end
end