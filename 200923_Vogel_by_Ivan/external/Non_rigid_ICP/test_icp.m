
% filename1 = sprintf('hand_target1_15000.ply');
% filename2 = sprintf('hand_template_new_15000.ply');
% [ListVertex1, ListFace1, ListFace_backup1, HEADER1, no_HEADER_lines1, no_Vertex1, no_Face1] = function_loading_ply_file(filename1);
% [ListVertex2, ListFace2, ListFace_backup2, HEADER2, no_HEADER_lines2, no_Vertex2, no_Face2] = function_loading_ply_file(filename2);

iterations = 10; % number of iterations; usually between 10 en 30
flag_prealligndata = 1;
    %  0 if the data still need to be roughly alligned
    %  1 if the data is already alligned (manual or landmark based)
    load after_non_rigid.mat
    load face_target_template.mat
    
    ListVertex1 = V;
    
    z = Results_NCPD(1:end,3);
    z = z+200;
    Results_NCPD_new = [Results_NCPD(1:end,1) Results_NCPD(1:end,2) z] 
    ListVertex2 = Results_NCPD_new;

    ListFace1 = F;
    ListFace2 = m;

nonrigidICP(ListVertex1,ListVertex2,ListFace1,ListFace2,iterations,flag_prealligndata); %targetV,templateV,targetF,templateF
