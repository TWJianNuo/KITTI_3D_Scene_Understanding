function [velo, intrinsic, extrinsic] = read_velo(base_dir,calib_dir,frame)
    cam = 2; % 0-based index
    th = 0;
    
    % load calibration
    calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
    Tr_velo_to_cam = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'));
    
    % compute projection matrix velodyne->image plane
    R_cam_to_rect = eye(4);
    R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
    P_velo_to_img = calib.P_rect{cam+1}*R_cam_to_rect*Tr_velo_to_cam;
    b = R_cam_to_rect*Tr_velo_to_cam;
    a = calib.P_rect{cam+1}; a(4,4) = 1;
    c = [1 0 0 a(1,4)/a(1,1); 0 1 0 a(2,4)/a(2,2); 0 0 1 0; 0 0 0 1];
    d = [a(1:3,1:3) [0; 0; 0]; [0 0 0 1]];
    
    intrinsic = d;
    extrinsic = c * b;
    % This transformation will introduce 0.4762 error of quadratic image
    % pixel location error for 63152 valid points
    
    % load and display image
    % img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,frame));
    % fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
    % figure(1); clf; imshow(img); hold on;
    
    % Display velodyne points
    fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin',base_dir,frame),'rb');
    velo = fread(fid,[4 inf],'single')';
    velo = velo(velo(:,1)>th,:); velo = [velo(:,1:3), ones(size(velo,1),1)];
    fclose(fid);
    %{
    selector = velo(:,1) > 0;
    velo_img = project(velo(selector,1:3),P_velo_to_img);
    velo_img = round(velo_img);
    selector_ = (velo_img(:,1) > 0) & (velo_img(:,1) < size(img, 2)) & (velo_img(:,2) > 0) & (velo_img(:,2) < size(img, 1));
    velo_img_in = velo_img(selector_, :); selector(selector) = selector_;
    velo_linear = sub2ind([size(img,1) size(img,2)], velo_img_in(:,2), velo_img_in(:,1));
    r = img(:,:,1); g = img(:,:,2); b = img(:,:,3);
    colors = [r(velo_linear) g(velo_linear) b(velo_linear)];
    colors = double(colors) / 255; 
    velo_3d = velo(selector, :);
    %}
    % figure(1); scatter(velo_img_in(:,1),velo_img_in(:,2),1);
    % figure(2); clf; scatter3(velo(selector,1), velo(selector,2), velo(selector,3), 1, colors); axis equal
end

function calib = loadCalibrationCamToCam(filename)
    
    % open file
    fid = fopen(filename,'r');
    
    if fid<0
        calib = [];
        return;
    end
    
    % read corner distance
    calib.cornerdist = readVariable(fid,'corner_dist',1,1);
    
    % read all cameras (maximum: 100)
    for cam=1:100
        
        % read variables
        S_      = readVariable(fid,['S_' num2str(cam-1,'%02d')],1,2);
        K_      = readVariable(fid,['K_' num2str(cam-1,'%02d')],3,3);
        D_      = readVariable(fid,['D_' num2str(cam-1,'%02d')],1,5);
        R_      = readVariable(fid,['R_' num2str(cam-1,'%02d')],3,3);
        T_      = readVariable(fid,['T_' num2str(cam-1,'%02d')],3,1);
        S_rect_ = readVariable(fid,['S_rect_' num2str(cam-1,'%02d')],1,2);
        R_rect_ = readVariable(fid,['R_rect_' num2str(cam-1,'%02d')],3,3);
        P_rect_ = readVariable(fid,['P_rect_' num2str(cam-1,'%02d')],3,4);
        
        % calibration for this cam completely found?
        if isempty(S_) || isempty(K_) || isempty(D_) || isempty(R_) || isempty(T_)
            break;
        end
        
        % write calibration
        calib.S{cam} = S_;
        calib.K{cam} = K_;
        calib.D{cam} = D_;
        calib.R{cam} = R_;
        calib.T{cam} = T_;
        
        % if rectification available
        if ~isempty(S_rect_) && ~isempty(R_rect_) && ~isempty(P_rect_)
            calib.S_rect{cam} = S_rect_;
            calib.R_rect{cam} = R_rect_;
            calib.P_rect{cam} = P_rect_;
        end
    end
    
    % close file
    fclose(fid);
end

function A = readVariable(fid,name,M,N)
    
    % rewind
    fseek(fid,0,'bof');
    
    % search for variable identifier
    success = 1;
    while success>0
        [str,success] = fscanf(fid,'%s',1);
        if strcmp(str,[name ':'])
            break;
        end
    end
    
    % return if variable identifier not found
    if ~success
        A = [];
        return;
    end
    
    % fill matrix
    A = zeros(M,N);
    for m=1:M
        for n=1:N
            [val,success] = fscanf(fid,'%f',1);
            if success
                A(m,n) = val;
            else
                A = [];
                return;
            end
        end
    end
end

function Tr = loadCalibrationRigid(filename)
    
    % open file
    fid = fopen(filename,'r');
    
    if fid<0
        error(['ERROR: Could not load: ' filename]);
    end
    
    % read calibration
    R  = readVariable(fid,'R',3,3);
    T  = readVariable(fid,'T',3,1);
    Tr = [R T;0 0 0 1];
    
    % close file
    fclose(fid);
end
