function [new_velo1, velo2] = align_img(img1, img2, velo1, velo2, intrin, extrin1, extrin2, rgbmask)
    reference_pt_num = 6; % As indicated by EPnP paper
    ran_num = 500;
    correspondence = find_cor_pts(img1, img2, rgbmask); % Find matched SIFT points
    % We align image1 to image2
    p1 = intrin * extrin1; p2 = intrin * extrin2;
    [p_velo1, dval1] = project(velo1,p1); [p_velo2, dval2] = project(velo2,p2);
    % visualize_velo(img1, velo1, p1);
    % visualize_velo(img2, velo2, p2);
    dep1 = griddata(p_velo1(:,1),p_velo1(:,2),dval1,correspondence(:,1),correspondence(:,2));
    dep2 = griddata(p_velo2(:,1),p_velo2(:,2),dval2,correspondence(:,3),correspondence(:,4));
    
    interpo_3d1 = back_project([correspondence(:,1), correspondence(:,2)], dep1, p1); % Get interpolcated 3D location of image1;
    interpo_3d2 = back_project([correspondence(:,3), correspondence(:,4)], dep2, p2); % Get interpolcated 3D location of image1;
    
    fin_extrinsic = RANSAC(correspondence, reference_pt_num, interpo_3d1, interpo_3d2, intrin, extrin2, ran_num);
    new_velo1 = (inv(extrin2) * fin_extrinsic * velo1')';
    % visualize_velo(img2, velo1, intrin * new_extrinsic_frame1);
    % figure(2); clf; scatter3(new_velo1(:,1),new_velo1(:,2),new_velo1(:,3),3,'r','fill');
    % hold on; scatter3(velo2(:,1),velo2(:,2),velo2(:,3),3,'g','fill')
    % verify_3d1 = back_project(p_velo1, dval1, p1);
    % verify_3d2 = back_project(p_velo2, dval2, p2);
    % visualize_velo(img1, verify_3d1, p1);
    % visualize_velo(img2, verify_3d2, p2);
    % interpo_3d1 = back_project([correspondence(:,1), correspondence(:,2)], dep1, p1);
    % interpo_3d2 = back_project([correspondence(:,3), correspondence(:,4)], dep2, p2);
    % r1 = img1(:,:,1); g1 = img1(:,:,2); b1 = img1(:,:,3); 
    % r2 = img2(:,:,1); g2 = img2(:,:,2); b2 = img2(:,:,3);
    % correspondence1_int = round(correspondence(:,5));
    % correspondence2_int = round(correspondence(:,6));
    % r1(correspondence1_int) = 255; g1(correspondence1_int) = 0; b1(correspondence1_int) = 0; 
    % r2(correspondence2_int) = 255; g2(correspondence2_int) = 0; b2(correspondence2_int) = 0;
    % img1_ = cat(3,r1,g1,b1); img2_ = cat(3,r2,g2,b2);
    
    % visualize_velo(img1, velo1, p1); 
    % hold on; scatter3(interpo_3d1(:,1),interpo_3d1(:,2),interpo_3d1(:,3),20,'r','fill')
    
    % For visualization
    visualize_velo2(img1, velo1, new_velo1, p1, img2, velo2, p2, true); % For visualization of 3D alignment results
    %{
    % Visualize corresponence between selected SIFT points
    img_ = [img1;img2];
    figure(2); clf; imshow(img_); hold on
    scatter(correspondence(:,1), correspondence(:,2),8,'r','fill'); hold on
    scatter(correspondence(:,3), correspondence(:,4) + size(img1,1),8,'r','fill');
    %}
end
function fin_extrinsic = RANSAC(correspondence, reference_pt_num, interpo_3d1, interpo_3d2, intrin, extrin2, ran_num)
    error_rec = zeros(ran_num,1); new_extrinsic_frame1_rec = cell(ran_num,1); ratio_fold = 4;
    for i = 1 : ran_num
        indexs = randperm(length(correspondence),reference_pt_num);
        [Rp,Tp,~,~]=efficient_pnp_gauss(interpo_3d1(indexs,:),[correspondence(indexs,3), correspondence(indexs,4)],intrin(1:3,1:3)); % Input 3D location of image2 and cooreponding 2D location of image 1
        new_extrinsic_frame1 = [Rp Tp;[0 0 0 1]]; new_extrinsic_frame1_rec{i} = new_extrinsic_frame1;
        new_velo1 = (inv(extrin2) * new_extrinsic_frame1 * interpo_3d1')';
        error = sum((new_velo1 - interpo_3d2).^2,2);
        s_error = sort(error);
        error_rec(i) = sum(s_error(1:ceil(end/ratio_fold)));
    end
    [~,idx] = min(error_rec); fin_extrinsic = new_extrinsic_frame1_rec{idx};
end
function correspondence = find_cor_pts(img1, img2, rgbmask)
    img1 = rgb2gray(img1); img2 = rgb2gray(img2);
    [fa,da] = vl_sift(single(img1));
    [fb,db] = vl_sift(single(img2));
    [matches, ~] = vl_ubcmatch(da, db); 
    image_size = look_up_specification('img_size');
    new_fa = fa(1:2,matches(1,:)); new_fb = fb(1:2,matches(2,:));
    lin_fa = sub2ind(image_size, round(new_fa(2,:)), round(new_fa(1,:)));
    lin_fb = sub2ind(image_size, round(new_fb(2,:)), round(new_fb(1,:)));
    selecor = rgbmask(lin_fa) & rgbmask(lin_fb);
    correspondence = [new_fa(:,selecor); new_fb(:,selecor); lin_fa(selecor); lin_fb(selecor)]';
    % img1(new_ind1) = 255; img2(new_ind2) = 255;
    % figure(1);clf;imshow(uint8(img1)); figure(2);clf;imshow(uint8(img2));
end
