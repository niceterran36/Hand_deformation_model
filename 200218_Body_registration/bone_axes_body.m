function axes = bone_axes_body(centers)
%BONE_AXES Compute local axes (local to world transform matrix) for each bone
%
% Inputs
%  centers: 30 by 1 array of spheres or 30 by 3 double
%
% Outpus
%  axes: 18 by 1 array of 4 by 4 double
%
 
    % Extract sphere centers if needed
    if iscell(centers)
        tmp = zeros(21, 3);
        for i = 1 : 21
            tmp(i, :) = centers{i}.center;
        end
        centers = tmp;
    end

    % Compute axes for each bone
    axes = cell(1, 21);
    
    % shoulder adduction/abduction
    axes{11} = build(centers(11,:),centers(10,:)-centers(11,:),centers(14,:)-centers(11,:));
    n_shoulder_left = -bone_normal(centers(14,:),centers(11,:),centers(10,:),[0 -1 0]);
    n_elbow_left = bone_normal(centers(15,:),centers(14,:), centers(16,:), cross(centers(14,:)-centers(15,:),centers(16,:)-centers(15,:)));
%    axes{14} = build(centers(14,:), centers(11,:) - centers(14,:), n_shoulder_left);
    axes{14} = build(centers(14,:), centers(10,:) - centers(11,:),centers(14,:)-centers(11,:));
    axes{15} = build(centers(15,:), centers(16,:) - centers(15,:), -n_elbow_left);
    axes{16} = build(centers(16,:), centers(16,:) - centers(15,:),n_elbow_left);
    
end

function normal = bone_normal(u, v, w, x)
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