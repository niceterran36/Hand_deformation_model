function [V, normals] = skin_linear_body2(V, weights, normals, transforms)

    % The normals are computed with the transpose inverse of the transforms
%     transformsNormals = cell(1, length(transforms));
%     for i = 1:length(transforms)
%         transformsNormals{i} = inv(transforms{i})';
%     end
    
    % Update vertices and normals
    for i = 1 : size(V, 1)
        t = interpolate(transforms, weights(i, :));
        V(i, :) = matrix_apply(t,V(i, :));
        t = interpolate(transformsNormals, weights(i, :));
       % normals(i, :) = matrix_apply(t, normals(i, :));
    end
  %  normals = normalizerow(normals);
    

%     if isfield(mesh, 'COR_bone')
%         %for i = 1 : length(mesh.COR_bone)
%         for i = 14:17
%             mesh.COR(i,:) = matrix_apply(transforms{mesh.COR_bone(i)}, mesh.COR(i,:));
%         end
%     end
end

function transform = interpolate(transforms, weights)
    transform = eye(4);
    for i = 1 : length(transforms)
        transform = transform + transforms{i} * weights(i);
    end
end
