function visualize_velo(rgb, velo, T)
    [p_out, depth, selector] = project(velo,T,true);
    p_out = round(p_out); image_size = look_up_specification('img_size');
    linaer_2d = sub2ind(image_size, p_out(:,2), p_out(:,1));
    color = zeros(length(depth), 3); r = rgb(:,:,1); g = rgb(:,:,2); b = rgb(:,:,3);
    color(:,1) = r(linaer_2d); color(:,2) = g(linaer_2d); color(:,3) = b(linaer_2d); color = color / 255;
    figure(1); clf; scatter3(velo(selector,1), velo(selector,2), velo(selector,3), 3, color, 'fill'); axis equal;
end