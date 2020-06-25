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
    
    % torso motion
    axes{11} = build(centers(11,:),centers(10,:)-centers(11,:),centers(14,:)-centers(11,:));
    % neck motion
    axes{12} = build(centers(11,:),centers(14,:)-centers(18,:),centers(10,:)-centers(11,:));
    
    % left leg motion
    n_knee_left = bone_normal(centers(3,:),centers(2,:), centers(4,:), cross(centers(2,:)-centers(3,:),centers(4,:)-centers(3,:)));
    axes{2} = build(centers(2,:), -(centers(2,:)-centers(6,:)), cross(centers(1,:)-centers(10,:),centers(2,:)-centers(6,:)));
    axes{3} = build(centers(3,:), centers(4,:) - centers(13,:), -n_knee_left);
    axes{4} = build(centers(4,:), cross(centers(4,:) - centers(3,:),-n_knee_left),-n_knee_left);

    % right leg motion
    n_knee_right = bone_normal(centers(7,:),centers(6,:), centers(8,:), cross(centers(6,:)-centers(7,:),centers(8,:)-centers(7,:)));
    axes{6} = build(centers(6,:), -(centers(2,:)-centers(6,:)), cross(centers(1,:)-centers(10,:),centers(2,:)-centers(6,:)));
    axes{7} = build(centers(7,:), centers(8,:) - centers(7,:), -n_knee_right);
    axes{8} = build(centers(8,:), cross(centers(8,:) - centers(7,:),-n_knee_left),-n_knee_right);
    
    % left arm motion
    n_elbow_left = bone_normal(centers(15,:),centers(14,:), centers(16,:), cross(centers(14,:)-centers(15,:),centers(16,:)-centers(15,:)));
    axes{14} = build(centers(14,:), -(centers(14,:)-centers(11,:)), cross(centers(14,:)-centers(11,:),centers(11,:)-centers(10,:)));
    axes{15} = build(centers(15,:), centers(16,:) - centers(15,:), -n_elbow_left);
    axes{16} = build(centers(16,:), cross(centers(16,:) - centers(15,:),-n_elbow_left),-n_elbow_left);
    % things to do: wrist ulnar, radial points identification 
    % --> new axis define for wrist motion
    
    % right arm motion
    n_elbow_right = bone_normal(centers(19,:),centers(18,:), centers(20,:), cross(centers(18,:)-centers(19,:),centers(20,:)-centers(19,:)));
    axes{18} = build(centers(18,:), (centers(18,:)-centers(11,:)), cross(centers(18,:)-centers(11,:),centers(11,:)-centers(10,:)));
    axes{19} = build(centers(19,:), centers(20,:) - centers(19,:), -n_elbow_right);
    axes{20} = build(centers(20,:), cross(centers(20,:) - centers(19,:),-n_elbow_left),-n_elbow_right);
    % things to do: wrist ulnar, radial points identification
    
    
    % things to do: define of supination, pronation axes for upper arms (left,right), thighs (left, right)
    
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