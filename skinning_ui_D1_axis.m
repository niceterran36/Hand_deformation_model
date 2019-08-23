function skinning_ui_D1_axis()

    % Register gptoolbox
    addpath(genpath('external'));

    % Load mesh
    mesh = load('mesh/neutral.mat');
    mesh = mesh.mesh;
    
    % Compute axis and initial transforms
    transforms = cell(1, 18);
    for i = 1 : 18
        transforms{i} = eye(4);
    end
    axes = bone_axes(mesh.spheres);
    
    % axis visualization
%     axes_1 = zeros(4,4); axes_1_line = zeros(6,3);
%     axes_2 = zeros(4,4); axes_2_line = zeros(6,3);
%     axes_3 = zeros(4,4); axes_3_line = zeros(6,3);
%     axes_4 = zeros(4,4); axes_4_line = zeros(6,3);
%     axes_5 = zeros(4,4); axes_5_line = zeros(6,3);
%     axes_6 = zeros(4,4); axes_6_line = zeros(6,3);
%     axes_7 = zeros(4,4); axes_7_line = zeros(6,3);
%     axes_8 = zeros(4,4); axes_8_line = zeros(6,3);
%     axes_9 = zeros(4,4); axes_9_line = zeros(6,3);
%     axes_10 = zeros(4,4); axes_10_line = zeros(6,3);
%     axes_11 = zeros(4,4); axes_11_line = zeros(6,3);
%     axes_12 = zeros(4,4); axes_12_line = zeros(6,3);
%     axes_13 = zeros(4,4); axes_13_line = zeros(6,3);
%     axes_14 = zeros(4,4); axes_14_line = zeros(6,3);
%     axes_15 = zeros(4,4); axes_15_line = zeros(6,3);
%     axes_16 = zeros(4,4); axes_16_line = zeros(6,3);
%     axes_17 = zeros(4,4); axes_17_line = zeros(6,3);
%     axes_18 = zeros(4,4); axes_18_line = zeros(6,3);
    
axes_3 = axes{1,3};
axes_3(1:3,1:3) = axes_3(1:3,1:3) + axes_3(1:3,4);
axes_3_line = zeros(6,3);
axes_3_line(1,1:3) = axes_3(1,1:3);
axes_3_line(3,1:3) = axes_3(2,1:3);
axes_3_line(5,1:3) = axes_3(3,1:3);
axes_3_line(2,1:3) = axes_3(1,4);
axes_3_line(4,1:3) = axes_3(2,4);
axes_3_line(6,1:3) = axes_3(3,4);

    
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
    
    % Render mesh
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

    scatter3(axes_3(1,1),axes_3(2,1),axes_3(3,1),'.', 'MarkerEdgeColor',[255/255, 0, 0]) % x-axis, colored red
    plot3(axes_3_line(1:2,1),axes_3_line(3:4,1),axes_3_line(5:6,1),'-r')
    scatter3(axes_3(1,2),axes_3(2,2),axes_3(3,2),'.', 'MarkerEdgeColor',[0, 255/255, 0]) % y-axis, colored green
    plot3(axes_3_line(1:2,2),axes_3_line(3:4,2),axes_3_line(5:6,2),'-g')
    scatter3(axes_3(1,3),axes_3(2,3),axes_3(3,3),'.', 'MarkerEdgeColor',[0, 0, 255/255]) % z-axis, colored blue
    plot3(axes_3_line(1:2,3),axes_3_line(3:4,3),axes_3_line(5:6,3),'-b')
    scatter3(axes_3(1,4),axes_3(2,4),axes_3(3,4),'.', 'MarkerEdgeColor',[0, 0, 0]) % center, colored black
    %scatter3(0,0,0,'.','MarkerEdgeColor',[0, 255/255, 222/255]) % origin(0,0,0), colored cyan

    hold off
    % Show figure
    set(f, 'Visible', 'on')
    
    % Update callback
    function update(varargin) % index finger 
        transforms{3} = matrix_rotation( ... % D1 CMC
            get(sliders{1}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{3}(1 : 3, 1)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{4} = matrix_rotation( ... % D1 MCP
            get(sliders{2}, 'Value'), ...
            matrix_apply(transforms{3}, axes{4}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{3}, axes{4}(1 : 3, 4)') ...
        ) * transforms{3};
        transforms{5} = matrix_rotation( ... % D1 IP
            get(sliders{3}, 'Value'), ...
            matrix_apply(transforms{4}, axes{5}(1 : 3, 2)', 0), ...
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

