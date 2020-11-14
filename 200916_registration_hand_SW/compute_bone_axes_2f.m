function axes = compute_bone_axes_2f(centers)
%COMPUTE_BONE_AXES Compute local axes for each bone
    

    tmp = zeros(10, 3);
    for i = 1:10
        tmp(i, :) = centers(i, :);
    end
    centers = tmp;

    axes = cell(1, 8);
    
    % Palm
    root = centers(1, :) + [0 0 -30];
    axes{1} = build(root, centers(1, :) - root, centers(1, :) + [10 0 0]);
    axes{2} = build(centers(1, :), centers(10, :) - centers(1, :), centers(1, :) + [10 0 0]);
    
    % Thumb
    % TODO is thumb normal correct?
    n_thumb = -finger_normal(centers(2, :), centers(3, :), centers(5, :), centers(6, :));
    axes{3} = build(centers(2, :), centers(3, :) - centers(2, :), n_thumb);
    axes{4} = build(centers(3, :), centers(4, :) - centers(3, :), n_thumb);
    axes{5} = build(centers(4, :), centers(5, :) - centers(4, :), n_thumb);
    
    % Index
    n_index = -finger_normal(centers(6, :), centers(7, :), centers(9, :), centers(10, :));
    axes{6} = build(centers(6, :), centers(7, :) - centers(6, :), n_index);
    axes{7} = build(centers(7, :), centers(8, :) - centers(7, :), n_index);
    axes{8} = build(centers(8, :), centers(9, :) - centers(8, :), n_index);

%     axes{9} = build(centers(8, :), centers(9, :) - centers(8, :), n_index);
%     axes{10} = build(centers(8, :), centers(9, :) - centers(8, :), n_index);
end

function normal = finger_normal(u, v, w, x)
    uv = normalizerow(v - u);
    uw = normalizerow(w - u);
    ux = normalizerow(x - u);
    n = cross(uw, uv);
    l = norm(n);
    if l < 0.001
        n = ux;
    else
        n = n / l;
    end
    if dot(ux, n) < 0
        n = -n;
    end
    normal = l * n + (1 - l) * ux;
end

function axe = build(o, x, y)
    x = normalizerow(x);
    y = normalizerow(y);
    z = normalizerow(cross(x, y));
    y = cross(z, x);
    axe = [x', y', z', o'; 0, 0, 0, 1];
end
