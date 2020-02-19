function mesh = skin_linear_body2(mesh, transforms)

    % The normals are computed with the transpose inverse of the transforms
    transformsNormals = cell(1, length(transforms));
    for i = 1:length(transforms)
        transformsNormals{i} = inv(transforms{i})';
    end
    
    % Update vertices and normals
    for i = 1 : size(mesh.V, 1)
        t = interpolate(transforms, mesh.weights(i, :));
        mesh.V(i, :) = matrix_apply(t,mesh.V(i, :));
        t = interpolate(transformsNormals, mesh.weights(i, :));
        mesh.normals(i, :) = matrix_apply(t, mesh.normals(i, :));
    end
    mesh.normals = normalizerow(mesh.normals);
    

    if isfield(mesh, 'COR_bone')
        %for i = 1 : length(mesh.COR_bone)
        for i = 14:17
            mesh.COR(i,:) = matrix_apply(transforms{mesh.COR_bone(i)}, mesh.COR(i,:)); % 문제 발생
        end
    end

%     Original format
%     if isfield(mesh, 'spheres')
%         for i = 1 : length(mesh.spheres)
%             mesh.spheres{i}.center = matrix_apply(transforms{mesh.spheres{i}.bone}, mesh.spheres{i}.center);
%         end
%     end
%     
end

function transform = interpolate(transforms, weights)
    transform = zeros(4);
    for i = 1 : length(transforms)
        transform = transform + transforms{i} * weights(i);
    end
end
