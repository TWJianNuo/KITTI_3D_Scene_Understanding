function rgb = read_rgb(base_dir, frame)
    cam = 2; % 0-based cam index
    path = sprintf('%s/image_%02d/data/%010d.png', base_dir, cam, frame);
    rgb = imread(path);
end