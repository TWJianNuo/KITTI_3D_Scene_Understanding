function generate_video_()
    base_dir = {
        % '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/output_results/SYNTHIA-SEQS-05-SPRING/metric/depth_map/';
        % '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/output_results/SYNTHIA-SEQS-05-SPRING/metric/reconstructed_diif_map/';
        % '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/output_results/SYNTHIA-SEQS-05-SPRING/metric/reconstructed_map/';
        '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/KITTI_3D_scene_reconstruction/inter_results/instance_level_seg/';
        % '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/output_results/SYNTHIA-SEQS-05-SPRING/metric/reconstructed_glob_map/';
        % '/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Synthia_3D_scenen_reconstruction_standalone/output_results/SYNTHIA-SEQS-05-SPRING/metric';
        };
    num = 106; image_size = look_up_specification('img_size');
    DateString = datestr(datetime('now')); DateString = strrep(DateString,'-','_');DateString = strrep(DateString,' ','_');DateString = strrep(DateString,':','_'); DateString = DateString(1:14);
    toy_object_video = VideoWriter([base_dir{1} '/' DateString]);
    toy_object_video.FrameRate = 6;
    open(toy_object_video);
    for i = 0 : num
        try
            img1 = imread([base_dir{1} num2str(i) '_1.png']); % img1 = imresize(img1, [image_size(1) image_size(2) / 2]);
            img2 = imread([base_dir{1} num2str(i) '_3.png']);
            % img1_ = uint8(zeros(466,142,3));
            img1 = imresize(img1, [466, ceil(size(img1,2)/size(img1,1)*466)]);
            img = [img1 img2];
            % img2 = imread([base_dir{1} num2str(i) '_2.png']); img2 = imresize(img2, [image_size(1) image_size(2) / 2]);
            % img3 = imread([base_dir{1} num2str(i) '_3.png']);
            % img = [img1 img2; img3];
            % img2 = imread([base_dir2 '_instance_label' num2str(i, '%d') '.png']);
            % img3 = imread([base_dir 'align_situation_' num2str(i, '%d') '.png']);
            % img1 = imresize(img1, [760, 1280]);
            % img3 = imresize(img3, [760, 1280]);
            % img = [img2; img1];
            writeVideo(toy_object_video, img)
        catch
            continue;
        end
    end
end
function [extrinsic_params, intrinsic_params, depth, label, instance, rgb] = grab_provided_data(frame)
    intrinsic_params = get_intrinsic_matrix();
    [base_path, GT_Depth_path, GT_seg_path, GT_RGB_path, GT_Color_Label_path, cam_para_path] = get_file_storage_path();
    f = num2str(frame, '%06d');
    txtPath = strcat(base_path, cam_para_path, num2str((frame-1), '%06d'), '.txt'); vec = load(txtPath); extrinsic_params = reshape(vec, 4, 4);
    ImagePath = strcat(base_path, GT_Depth_path, f, '.png'); depth = getDepth(ImagePath);
    ImagePath = strcat(base_path, GT_seg_path, f, '.png'); [label, instance] = getIDs(ImagePath);
    ImagePath = strcat(base_path, GT_RGB_path, f, '.png'); rgb = imread(ImagePath);
    % stored_mark = load(['/home/ray/ShengjieZhu/Fall Semester/depth_detection_project/Exp_re/segmentation_results/21_Sep_2018_07_segmentation/Instance_map/', f, '.mat']); stored_mark = stored_mark.prev_mark;
end