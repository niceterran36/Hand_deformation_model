function test_skinning_ui_newhand_D1()

    % Register gptoolbox
    addpath(genpath('external'));
    addpath 'functions';

%    mesh = load('hy_mesh.mat');
%    mesh2 = load('mesh/neutral.mat');
    mesh = load('hy_mesh_n.mat');
    mesh = mesh.mesh;

%     vertices = mesh.vertices;
%     faces = mesh.faces;
%     N = per_vertex_normals(vertices,faces);
%     mesh.normals = N;
    
%     % Load mesh
%     mesh.vertices = vertices;
%     mesh.faces = faces;
%     mesh.weights = weights;
%     mesh.normals = per_vertex_normals(vertices,faces);   
    
    % Compute axis and initial transforms
    transforms = cell(1, 18);
    for i = 1 : 18
        transforms{i} = eye(4);
    end
    axes = bone_axes(mesh.spheres);
    
    % translate the axes vector from origin to each axis center 
    axes_t = axes;
    for i = 1:18
    axes_t{1,i}(1:3,1:3) = axes_t{1,i}(1:3,1:3)+axes_t{1,i}(1:3,4);
    end     
    
    % variables of axis end-point & center-point 
    axes_x_pt = zeros(18,3); axes_y_pt = zeros(18,3); axes_z_pt = zeros(18,3); axes_center_pt = zeros(18,3);
        for i = 1:18
        axes_x_pt(i,:) = axes_t{1,i}(1:3)';
    end
    for i = 1:18
        axes_y_pt(i,:) = axes_t{1,i}(5:7)';
    end
    for i = 1:18
        axes_z_pt(i,:) = axes_t{1,i}(9:11)';
    end
    for i = 1:18
        axes_center_pt(i,:) = axes_t{1,i}(13:15)';
    end  
    
% figure(1)
% hold on
% axis equal
% axis off
% grid off
% % line plotting format plot3([A(1) B(1)],[A(2) B(2)],[A(3) B(3)], '-k')
%     for i = 1:18
%         plot3([axes_t{1,i}(1,1) axes_t{1,i}(1,4)],[axes_t{1,i}(2,1) axes_t{1,i}(2,4)],[axes_t{1,i}(3,1) axes_t{1,i}(3,4)], '-r')
%     end 
%     for i = 1:18
%         plot3([axes_t{1,i}(1,2) axes_t{1,i}(1,4)],[axes_t{1,i}(2,2) axes_t{1,i}(2,4)],[axes_t{1,i}(3,2) axes_t{1,i}(3,4)], '-g')
%     end 
%     for i = 1:18
%         plot3([axes_t{1,i}(1,3) axes_t{1,i}(1,4)],[axes_t{1,i}(2,3) axes_t{1,i}(2,4)],[axes_t{1,i}(3,3) axes_t{1,i}(3,4)], '-b')
%     end 
%     scatter3(axes_x_pt(:,1),axes_x_pt(:,2),axes_x_pt(:,3),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
%     scatter3(axes_y_pt(:,1),axes_y_pt(:,2),axes_y_pt(:,3),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
%     scatter3(axes_z_pt(:,1),axes_z_pt(:,2),axes_z_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
%     scatter3(axes_center_pt(:,1),axes_center_pt(:,2),axes_center_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
% hold off


    
    % Prepare components
    f = figure('Visible', 'off');
    popup = uicontrol( ...
        'Style', 'popup', ...
        'String', {'linear', 'dualquat', 'implicit'}, ...
        'Position', [20 20 100 20], ...
        'Callback', @update ...
    );
    sliders = cell(1, 15);
    for i = 1 : 15
        sliders{i} = uicontrol( ...
            'Style', 'slider', ...
            'Min', -1, 'Max', 2, 'Value', 0, ...
            'Position', [20, 25 * (16 - i) + 20, 100, 20], ...
            'Callback', @update ...
        );
    end 
    
    %% Render mesh with axis, center
    hold on
    h = trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
    view([-90, 0]);
    camlight;
    view([90, 0]);
    camlight;
    axis equal;
    grid off;
    lighting gouraud;
    axis off;
    
    % axis line drawing 
    for i = 1:18
        plot3([axes_t{1,i}(1,1) axes_t{1,i}(1,4)],[axes_t{1,i}(2,1) axes_t{1,i}(2,4)],[axes_t{1,i}(3,1) axes_t{1,i}(3,4)], '-r')
    end 
    for i = 1:18
        plot3([axes_t{1,i}(1,2) axes_t{1,i}(1,4)],[axes_t{1,i}(2,2) axes_t{1,i}(2,4)],[axes_t{1,i}(3,2) axes_t{1,i}(3,4)], '-g')
    end 
    for i = 1:18
        plot3([axes_t{1,i}(1,3) axes_t{1,i}(1,4)],[axes_t{1,i}(2,3) axes_t{1,i}(2,4)],[axes_t{1,i}(3,3) axes_t{1,i}(3,4)], '-b')
    end 
    % axis end-point, center-point drawing
    scatter3(axes_x_pt(:,1),axes_x_pt(:,2),axes_x_pt(:,3),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
    scatter3(axes_y_pt(:,1),axes_y_pt(:,2),axes_y_pt(:,3),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
    scatter3(axes_z_pt(:,1),axes_z_pt(:,2),axes_z_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
    scatter3(axes_center_pt(:,1),axes_center_pt(:,2),axes_center_pt(:,3),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
    %scatter3(0,0,0,'.','MarkerEdgeColor',[0, 255/255, 222/255]) % origin(0,0,0), colored cyan

    hold off
    
    %% Show figure
    set(f, 'Visible', 'on')
    
    % Update callback
    function update(varargin) % index finger 
        transforms{3} = matrix_rotation( ... % D1 CMC
            get(sliders{1}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg.
            matrix_apply(transforms{2}, axes{3}(1 : 3, 1)', 0), ... % axis1: red; axis2: green; axis3: blue
            matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{4} = matrix_rotation( ... % D1 MCP
            get(sliders{2}, 'Value'), ...
            matrix_apply(transforms{3}, axes{4}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
            matrix_apply(transforms{3}, axes{4}(1 : 3, 4)') ...
        ) * transforms{3};
        transforms{5} = matrix_rotation( ... % D1 IP
            get(sliders{3}, 'Value'), ...
            matrix_apply(transforms{4}, axes{5}(1 : 3, 2)', 0), ... % axis1: red; axis2: green; axis3: blue
            matrix_apply(transforms{4}, axes{5}(1 : 3, 4)') ...
        ) * transforms{4};
    
        angle_1 = get(sliders{1}, 'Value');
        angle_2 = get(sliders{2}, 'Value');
        angle_3 = get(sliders{3}, 'Value');

        transforms{6} = matrix_rotation( ... % D2 MCP
            get(sliders{4}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{6}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{6}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{7} = matrix_rotation( ... % D2 PIP
            get(sliders{5}, 'Value'), ...
            matrix_apply(transforms{6}, axes{7}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{6}, axes{7}(1 : 3, 4)') ...
        ) * transforms{6};
        transforms{8} = matrix_rotation( ... % D2 DIP
            get(sliders{6}, 'Value'), ...
            matrix_apply(transforms{7}, axes{8}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{7}, axes{8}(1 : 3, 4)') ...
        ) * transforms{7};
    
        angle_4 = get(sliders{4}, 'Value');
        angle_5 = get(sliders{5}, 'Value');
        angle_6 = get(sliders{6}, 'Value');
 
        transforms{9} = matrix_rotation( ...  % D3 MCP
            get(sliders{7}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{9}(1 : 3, 3)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{9}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{10} = matrix_rotation( ... % D3 PIP
            get(sliders{8}, 'Value'), ...
            matrix_apply(transforms{9}, axes{10}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{9}, axes{10}(1 : 3, 4)') ...
        ) * transforms{9};
        transforms{11} = matrix_rotation( ... % D3 DIP
            get(sliders{9}, 'Value'), ...
            matrix_apply(transforms{10}, axes{11}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{10}, axes{11}(1 : 3, 4)') ...
        ) * transforms{10};
    
        angle_7 = get(sliders{7}, 'Value');
        angle_8 = get(sliders{8}, 'Value');
        angle_9 = get(sliders{9}, 'Value');

        transforms{12} = matrix_rotation( ...  % D4 MCP
            get(sliders{10}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{12}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{12}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{13} = matrix_rotation( ... % D4 PIP
            get(sliders{11}, 'Value'), ...
            matrix_apply(transforms{12}, axes{13}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{12}, axes{13}(1 : 3, 4)') ...
        ) * transforms{12};
        transforms{14} = matrix_rotation( ... % D4 DIP
            get(sliders{12}, 'Value'), ...
            matrix_apply(transforms{13}, axes{14}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{13}, axes{14}(1 : 3, 4)') ...
        ) * transforms{13};
    
        angle_10 = get(sliders{10}, 'Value');
        angle_11 = get(sliders{11}, 'Value');
        angle_12 = get(sliders{12}, 'Value');

    
        transforms{16} = matrix_rotation( ... % D5 MCP
            get(sliders{13}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{16}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{16}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{17} = matrix_rotation( ...% D5 PIP
            get(sliders{14}, 'Value'), ...
            matrix_apply(transforms{16}, axes{17}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{16}, axes{17}(1 : 3, 4)') ...
        ) * transforms{16};
        transforms{18} = matrix_rotation( ... % D5 DIP
            get(sliders{15}, 'Value'), ...
            matrix_apply(transforms{17}, axes{18}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{17}, axes{18}(1 : 3, 4)') ...
        ) * transforms{17};
    
        angle_13 = get(sliders{13}, 'Value');
        angle_14 = get(sliders{14}, 'Value');
        angle_15 = get(sliders{15}, 'Value');
    
        
        disp([angle_1 angle_2 angle_3; angle_4 angle_5 angle_6; angle_7 angle_8 angle_9; angle_10 angle_11 angle_12;...
            angle_13 angle_14 angle_15;])
    
    
    
        transformed = mesh;
        switch get(popup, 'Value')
        case 1
            transformed = skin_linear(transformed, transforms);
        case 2
            transformed = skin_dualquat(transformed, transforms);
        case 3
            transformed = skin_implicit(transformed, transforms, 20, 'hrbf');
        end
        hold on;
        delete(h);
        h = trimesh(transformed.faces, transformed.vertices(:, 1), transformed.vertices(:, 2), transformed.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
        lighting gouraud;
        hold off;
    end

end

