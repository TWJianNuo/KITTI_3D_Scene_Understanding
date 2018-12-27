function align_img_demo()
    run('/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/Matlab_code/vlfeat-0.9.21/toolbox/vl_setup.m');
    % run "vl_setup" to register required sift functions to Matlab path
    % for more info about vl_sif, refer website: http://www.vlfeat.org/index.html
    calib_dir = '/home/ray/Downloads/KITTI_Raw/2011_09_26/';
    base_dir = '/home/ray/Downloads/KITTI_Raw/2011_09_26/2011_09_26_drive_0001_sync/';
    % Data is organized as specified in KITTI "raw data download script"
    % The script is available: http://www.cvlibs.net/datasets/kitti/raw_data.php
    for i = 0 : 0
        frame_1 = i; frame_2 = i + 1;
        img1 = read_rgb(base_dir, frame_1); img2 = read_rgb(base_dir, frame_2);
        [velo1, intrin, extrin1] = read_velo(base_dir,calib_dir,frame_1);
        [velo2, ~, extrin2] = read_velo(base_dir,calib_dir,frame_2);
        rgbmask = get_mask();
        [new_velo1, velo2] = align_img(img1, img2, velo1, velo2, intrin, extrin1, extrin2, rgbmask);
        % pts_3d = [velo2; new_velo1];
        figure(1); clf; scatter3(velo2(:,1),velo2(:,2),velo2(:,3),3,'r','fill')
        hold on; scatter3(new_velo1(:,1),new_velo1(:,2),new_velo1(:,3),3,'g','fill'); axis equal
    end
end


