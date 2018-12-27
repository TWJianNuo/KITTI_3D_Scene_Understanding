function return_info = look_up_specification(entry)
    speci_info = {
        'Dataset', 'KITTI';
        'img_size', [375, 1242];
        };
    return_info = speci_info{contains(speci_info(:,1),string(entry)),2};
end