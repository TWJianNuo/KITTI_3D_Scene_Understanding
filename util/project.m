function [p_out, depth, selector] = project(p_in,T,is_in_img,img_size)
    
    % dimension of data and projection matrix
    dim_norm = size(T,1);
    dim_proj = size(T,2);
    
    % do transformation in homogenuous coordinates
    p2_in = p_in;
    if size(p2_in,2)<dim_proj
        p2_in(:,dim_proj) = 1;
    end
    p2_out = (T*p2_in')';
    depth = p2_out(:,3);
    % normalize homogeneous coordinates:
    p_out = [p2_out(:,1) ./ depth, p2_out(:,2) ./ depth];
    
    if nargin > 2
        if is_in_img
            if nargin >= 3 && nargin < 4
                image_size = look_up_specification('img_size');
            elseif nargin == 4
                image_size = img_size;
            end
            p_outd = round(p_out);
            selector = (p_outd(:,1) > 0) & (p_outd(:,2) > 0) & (p_outd(:,1) < image_size(2)) & (p_outd(:,2) < image_size(1));
            p_out = p_out(selector,:);
            depth = depth(selector);
        end
    end
    
end