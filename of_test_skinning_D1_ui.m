function test_skinning_D1_ui()

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
    h = trimesh(mesh.faces, mesh.vertices(:, 1), mesh.vertices(:, 2), mesh.vertices(:, 3), 'EdgeColor', 'none', 'FaceColor', [0.8, 0.8, 0.8], 'FaceAlpha', 0.5);
    view([-90, 0]);
    camlight;
    view([90, 0]);
    camlight;
    axis equal;
    grid off;
    lighting gouraud;
    axis off;
    
    % Show figure
    set(f, 'Visible', 'on')
    
    % Update callback
    function update(varargin) % index finger 
        transforms{3} = matrix_rotation( ... % D1 CMC
            get(sliders{1}, 'Value'), ... % rotation angle: 0 ~ 2, range 3 = 180 deg. 
            matrix_apply(transforms{2}, axes{3}(1 : 3, 2)', 0), ... % axis - 1: supination, pronation; 2: flexion/extension; 3: abduction/adduction
            matrix_apply(transforms{2}, axes{3}(1 : 3, 4)') ... % center
        ) * transforms{2};
        transforms{4} = matrix_rotation( ... % D1 MCP
            get(sliders{2}, 'Value'), ... % maxima rotation angle = 50 deg. 
            matrix_apply(transforms{3}, axes{4}(1 : 3, 2)', 0), ...
            matrix_apply(transforms{3}, axes{4}(1 : 3, 4)') ...
        ) * transforms{3};
        transforms{5} = matrix_rotation( ... % D1 IP
            get(sliders{3}, 'Value'), ... % maxima rotation angle = 80 deg. 
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

