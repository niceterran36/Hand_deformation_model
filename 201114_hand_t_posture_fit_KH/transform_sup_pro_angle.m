function transformed = transform_sup_pro_angle(mesh, angle)

transforms = cell(1, 18);
for i = 1 : 18
    transforms{i} = eye(4);
end
axes = bone_axes(mesh.spheres);

transforms_sp = cell(1,4);
for i = 1 : 4
    transforms_sp{i} = eye(4);
end 

 % MCP abduction/adduction motion
transforms_sp{1} = matrix_rotation( ... % D2 MCP -sp/pr
    angle(1), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 1)', 0), ... axis - 1: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_sp{2} = matrix_rotation( ...  % D3 MCP - ab/ad
    angle(2), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{9}(1 : 3, 1)', 0), ... axis - 1: abduction/adduction
    matrix_apply(transforms{2}, axes{9}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_sp{3} = matrix_rotation( ...  % D4 MCP -ab/ad
    angle(3), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{12}(1 : 3, 1)', 0), ... axis - 1: abduction/adduction
    matrix_apply(transforms{2}, axes{12}(1 : 3, 4)') ... % center
) * transforms{2};
transforms_sp{4} = matrix_rotation( ... % D5 MCP - ab/ad
    angle(4), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{16}(1 : 3, 1)', 0), ...  axis - 1: abduction/adduction
    matrix_apply(transforms{2}, axes{16}(1 : 3, 4)') ... % center
) * transforms{2};

% Flexion/extension of fingers 
transforms{3} = matrix_rotation( ... % D1 CMC
    0, ... % rotation angle: 0 ~ 2, range 3 = 180 deg.
    matrix_apply(transforms{2}, axes{3}(1 : 3, 1)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ... % center
) * transforms{2};
transforms{4} = matrix_rotation( ... % D1 MCP
    0, ...
    matrix_apply(transforms{3}, axes{4}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{3}, axes{4}(1 : 3, 4)') ...
) * transforms{3};
transforms{5} = matrix_rotation( ... % D1 IP
    0, ...
    matrix_apply(transforms{4}, axes{5}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
    matrix_apply(transforms{4}, axes{5}(1 : 3, 4)') ...
) * transforms{4};

transforms{6} = matrix_rotation( ... % D2 MCP
    0, ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{6}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
) * transforms_sp{1} * transforms{2};
transforms{7} = matrix_rotation( ... % D2 PIP
    0, ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{6}, axes{7}(1 : 3, 4)') ...
) * transforms{6};
transforms{8} = matrix_rotation( ... % D2 DIP
    0, ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{7}, axes{8}(1 : 3, 4)') ...
) * transforms{7};

transforms{9} = matrix_rotation( ...  % D3 MCP
    0, ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{9}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{9}(1 : 3, 4)') ... % center
) * transforms_sp{2} * transforms{2};
transforms{10} = matrix_rotation( ... % D3 PIP
    0, ...
    matrix_apply(transforms{9}, axes{10}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{9}, axes{10}(1 : 3, 4)') ...
) * transforms{9};
transforms{11} = matrix_rotation( ... % D3 DIP
    0, ...
    matrix_apply(transforms{10}, axes{11}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{10}, axes{11}(1 : 3, 4)') ...
) * transforms{10};

transforms{12} = matrix_rotation( ...  % D4 MCP
    0, ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{12}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{12}(1 : 3, 4)') ... % center
) * transforms_sp{3}* transforms{2};
transforms{13} = matrix_rotation( ... % D4 PIP
    0, ...
    matrix_apply(transforms{12}, axes{13}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{12}, axes{13}(1 : 3, 4)') ...
) * transforms{12};
transforms{14} = matrix_rotation( ... % D4 DIP
    0, ...
    matrix_apply(transforms{13}, axes{14}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{13}, axes{14}(1 : 3, 4)') ...
) * transforms{13};

transforms{16} = matrix_rotation( ... % D5 MCP
    0, ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
    matrix_apply(transforms{2}, axes{16}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
    matrix_apply(transforms{2}, axes{16}(1 : 3, 4)') ... % center
) * transforms_sp{4} * transforms{2};
transforms{17} = matrix_rotation( ...% D5 PIP
    0, ...
    matrix_apply(transforms{16}, axes{17}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{16}, axes{17}(1 : 3, 4)') ...
) * transforms{16};
transforms{18} = matrix_rotation( ... % D5 DIP
    0, ...
    matrix_apply(transforms{17}, axes{18}(1 : 3, 2)', 0), ...
    matrix_apply(transforms{17}, axes{18}(1 : 3, 4)') ...
) * transforms{17};

%Apply new hand posture by angle & render 3D model   

transformed = skin_dualquat(mesh, transforms);
for i = 1:18
     transformed.centers(i,:) = transformed.spheres{1,i}.center;
end

end 