function test_skinning_ui_newhand_D1()

    % Register gptoolbox
    addpath(genpath('external'));
    mesh = load('hy_mesh.mat');
%     mesh = load('mesh/neutral.mat');
    mesh = mesh.mesh;

%     % Load mesh
%     mesh.vertices = vertices;
%     mesh.faces = faces;
%     mesh.weights = weights;
%     mesh.normals = getNormals(vertices,faces);  
    
    % Compute axis and initial transforms
    transforms = cell(1, 18);
    for i = 1 : 18
        transforms{i} = eye(4);
    end
    axes = bone_axes(mesh.spheres);
    
    % translate the axes vector from origin to each axis center 
    axes_t = axes;
    for i = 1:18;
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
    sliders = cell(1, 3);
    for i = 1 : 3
        sliders{i} = uicontrol( ...
            'Style', 'slider', ...
            'Min', -1, 'Max', 2, 'Value', 0, ...
            'Position', [20, 25 * (4 - i) + 20, 100, 20], ...
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

