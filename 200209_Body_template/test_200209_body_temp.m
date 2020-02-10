addpath('F:\[GitHub]\Hand_deformation_model\functions');
addpath(genpath('external'));

% [V1, F1, FB1, Header1] = function_loading_ply_file('newTemplate_body_WS_200209.ply'); % Wonsup's template original
%V2 = V1+[0 145.758 867.945]; % vertex translation for modification

[V, F, FB1, Header1] = function_loading_ply_file('newTemplate_body_HY_200209.ply'); % modified template (vertex location)
%load('COR.mat'); % original template COR
load('COR_new.mat');
load('Segment_joint.mat');  % segment link information
V_idx = [1:size(V,1)]';
V2 = V;
VLI = [];

C = COR_new;
A = V;

figure()
hold on
axis equal
scatter3(A(:,1),A(:,2),A(:,3),'.', 'MarkerEdgeColor',[180/255, 180/255, 180/255])
plot3(C(:,1),C(:,2),C(:,3),'b*')
plot3(C([1 10 11 12 13],1),C([1 10 11 12 13],2),C([1 10 11 12 13],3), 'k-');
plot3(C(1:5,1),C(1:5,2),C(1:5,3), 'k-');
plot3(C([1 6:9],1),C([1 6:9],2),C([1 6:9],3), 'k-');
plot3(C(14:17,1),C(14:17,2),C(14:17,3), 'k-');
plot3(C(18:21,1),C(18:21,2),C(18:21,3), 'k-');
plot3(C([14 11 18],1),C([14 11 18],2),C([14 11 18],3), 'k-');
hold off

%% segmentation

tic

for vertexIdx = 1:size(V,1);

F2 = F;
LI = F2 == vertexIdx;
[row2, col2] = find(LI);
F2 = F2(row2,:);

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
    normals_F = normals(row2,:);
    normals_F = sum(normals_F);
    normals_F = normals_F/norm(normals_F);
    vni = normals_F; % weighted normal vector of vi

    Sj = zeros(20,5); Sj(1:20,1) = [1:20]';

    for segment=1:20

        vt = V(vertexIdx,:);
        jna = A(S(segment,1),:); jnb = A(S(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        Sj(segment,2) = delta;

        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        %dv1 = sqrt(norm(av)^2 - norm(cv)^2);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sj(segment,3) = dv2;
        dv = dv./dv2;
        cosTH = dot(dv,vni)/(norm(dv)*norm(vni));
        Sj(segment,4) = cosTH;

        % min-distance
        if delta >= 0 && delta <= 1
           min_d = dv2;
        elseif delta < 0
           min_d = norm(vt-jna);
        else
           min_d = norm(vt-jnb);
        end
        Sj(segment,5) = min_d;

    end

Sj_c = Sj;

    if size(Sj_c,1) ~= 0
        LIX = Sj_c(:,4)>=0; %angle condition1
        Sj_c = Sj(LIX,:);

        if size(Sj_c,1) ~= 0
            LIX = Sj_c(:,4)<1; % angle condition2
            Sj_c = Sj_c(LIX,:);
            
            if size(Sj_c,1) ~= 0
                LIX = Sj_c(:,2)>=0; % cut with delta 
                Sj_c = Sj_c(LIX,:);
                
                if size(Sj_c,1) ~= 0
                    LIX = Sj_c(:,2)<1; % cut with delta
                    Sj_c = Sj_c(LIX,:);
                                        
                    if size(Sj_c,1) ~= 0
                        LIX = Sj_c(:,5)<17; % cut with distance
                        Sj_c = Sj_c(LIX,:);
                        
                        if size(Sj_c,1) == 0;
                            v_segment(vertexIdx,1) = 21;
                        else
                            v_mindist = min(Sj_c(:,3));
                            [row,col] = find(Sj_c(:,3) == v_mindist); % searching min distance
                            v_delta = Sj_c(row,2); % final delta of vi
                            v_segment(vertexIdx,1) = Sj_c(row,1); % final segment of vi
                        end
                        
                    else
                        v_segment(vertexIdx,1) = 21;
                    end
                    
                else
                    v_segment(vertexIdx,1) = 21;
                end
                
            else
                v_segment(vertexIdx,1) = 21;
            end
            
        else
            v_segment(vertexIdx,1) = 21;
        end

    else
        v_segment(vertexIdx,1) = 21;
    end  
    
 v_mindist_s = min(Sj(:,5)); % cut with distance
% [row,col] = find(Sj(:,5) == v_mindist_s);
% LIX = Sj_c(:,5) == v_mindist_s;
% Sj_1 = Sj_c(:,1); Sj_2 = Sj_c(:,2); Sj_3 = Sj_c(:,3); Sj_4 = Sj_c(:,4); Sj_5 = Sj_c(:,5);
% Sj_c = [Sj_1(LIX) Sj_2(LIX) Sj_3(LIX) Sj_4(LIX) Sj_5(LIX)];

end


% segment 21 treatment 
% Sg21 = [idx, x, y, z]

Sg21 = []; 
temLI = v_segment == 21;  Sg21(:,2:4) =  V2(temLI,:);
Sg21(:,1) = V_idx(temLI);


for i = 1:size(Sg21,1)

    vertexIdx = Sg21(i);
%     vertexIdx = 17

        Sz = zeros(20,3); Sz(1:20,1) = [1:20]';

    for segment=1:20

        % projection-distance (Sz(:,2))
        vt = V(vertexIdx,:);
        jna = A(S(segment,1),:); jnb = A(S(segment,2),:); 
        vt_jna = vt - jna;
        jnb_jna = jnb - jna;
        delta = dot(vt_jna,jnb_jna)/ (norm(jnb-jna))^2;
        av = vt-jna; 
        bv = jnb-jna; 
        cv = dot(av,bv)/norm(bv);
        dv = av - dot(av,bv)/norm(bv)^2 * bv; % projection vector
        dv2 = norm(dv);
        Sz(segment,2) = dv2;
        Sz(segment,4) = delta;
        
        % min-distance (Sz(:,3))
        if delta >= 0 && delta <= 1
           min_d = dv2;
        elseif delta < 0
           min_d = norm(vt-jna);
        else
           min_d = norm(vt-jnb);
        end
        Sz(segment,3) = min_d;

    end
    
    v22_mindist1 = min(Sz(:,3));
    [row3,col3] = find(Sz(:,3) == v22_mindist1);
        
    if size(row3,1) == 1
        v_segment(vertexIdx,1) = Sz(row3,1);
    else
        v22_mindist2 = [Sz(row3(1),2); Sz(row3(end),2)];
        v22_mindist2 = max(v22_mindist2);
        [row4,col4] = find(Sz(:,2) == v22_mindist2);
        v_segment(vertexIdx,1) = Sz(row4,1);
    end
    
end     
toc