function [affine_matrx, aligned_pts_3d, gplane_param] = gplane_align(pts_3d, intrinsic, extrinsic)
    % Ground plane estimation will substitute coordinate system and
    % the 3d points
    
    % In this function, two way of ground plane estimation is provided:
    % 1. Strict geometry transformation
    % 2. vertical shitf of points
    
    [gplane_param, inlierIndices] = gplane_est(pts_3d);
    
    % Alignment method1:
    affine_matrx = get_affine_transformation_from_plane(gplane_param);
    aligned_pts_3d = (affine_matrx * pts_3d')';

    
    % Alignment method2:
    ver_shift = mean(pts_3d(inlierIndices,3));
    affine_matrx = [eye(3) [0 0 -ver_shift]'; [0 0 0 1];];
    aligned_pts_3d = (affine_matrx * pts_3d')';
    
    % Some visualization:
    figure(1); clf; scatter3(aligned_pts_3d(:,1),aligned_pts_3d(:,2),aligned_pts_3d(:,3),3,'r','fill'); axis equal;
    draw_ground_plane(1, gplane_param);
end
function pt = rand_sample_pt_on_plane(param, flag)
    % Sample points on plane given in param, flag indicates which
    % pre-defined point x-y coordinate value to use
    if flag
        pt = [0 1];
    else
        pt = [1 0];
    end
    pt = [pt, - (param(1) * pt(1) + param(2) * pt(2) + param(4)) / param(3)];
end
function affine_transformation = get_affine_transformation_from_plane(plane_param)
    origin = [0 0 0];
    dir1 = (rand_sample_pt_on_plane(plane_param, true) - rand_sample_pt_on_plane(plane_param, false)); dir1 = dir1 / norm(dir1);
    dir3 = plane_param(1:3); dir3 = dir3 / norm(dir3);
    dir2 = cross(dir1, dir3); dir2 = dir2 / norm(dir2);
    dir =[dir1;dir2;dir3];
    affine_transformation = get_affine_transformation(origin, dir);
end
function affine_transformation = get_affine_transformation(origin, new_basis)
    pt_camera_origin_3d = origin;
    x_dir = new_basis(1, :);
    y_dir = new_basis(2, :);
    z_dir = new_basis(3, :);
    new_coord1 = [1 0 0];
    new_coord2 = [0 1 0];
    new_coord3 = [0 0 1];
    new_pts = [new_coord1; new_coord2; new_coord3];
    old_Coord1 = pt_camera_origin_3d + x_dir;
    old_Coord2 = pt_camera_origin_3d + y_dir;
    old_Coord3 = pt_camera_origin_3d + z_dir;
    old_pts = [old_Coord1; old_Coord2; old_Coord3];
    
    T_m = new_pts' * inv((old_pts - repmat(pt_camera_origin_3d, [3 1]))');
    transition_matrix = eye(4,4);
    transition_matrix(1:3, 1:3) = T_m;
    transition_matrix(1, 4) = -pt_camera_origin_3d * x_dir';
    transition_matrix(2, 4) = -pt_camera_origin_3d * y_dir';
    transition_matrix(3, 4) = -pt_camera_origin_3d * z_dir';
    affine_transformation = transition_matrix;
end
function draw_ground_plane(figure_ind, params)
    xl = xlim; yl = ylim; 
    pts = [xl(1) yl(1); xl(1) yl(2); xl(2) yl(2); xl(2) yl(1)];
    zz = (-params(1) * pts(:,1) - params(2) * pts(:,2) - params(4)) / params(3);
    pts = [pts zz];
    figure(figure_ind); hold on; patch('Faces',[1 2 3 4],'Vertices',pts,'FaceAlpha', 0.5)
end