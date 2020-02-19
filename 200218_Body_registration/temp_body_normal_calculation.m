function N_P_V = temp_body_normal_calculation(V,F)

N_P_V = zeros(size(V,1),3); % N_P_V: normal per vertex

    for i = 1:size(V,1) 
      vertexIdx = i;

    F2 = F;
    LI = F2 == vertexIdx;
    [row, ~] = find(LI);
    F2 = F2(row,:);

        for i = 1:size(F2,1)*3
            v(i,1) = F2(i);
        end

        nv = vertexIdx;
        for i = 1:size(v,1)
            if v(i) ~= nv
               nv = [nv v(i)]; % nv: List of neighbor vertices of target with 1-node
            end
        end
        normals = getNormals(V, F);
        normals_F = normals(row,:);
        normals_F = sum(normals_F);
        normals_F = normals_F/norm(normals_F);
        N_P_V(vertexIdx,:) = normals_F;

    end 

end 