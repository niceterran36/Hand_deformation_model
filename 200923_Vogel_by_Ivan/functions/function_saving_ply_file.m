function function_saving_ply_file(ListVertex, ListFace_backup, HEADER, filename_save)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab Tutorial for 3D Anthropometry
    %
    % Wonsup Lee
    % 17 Sep 2015
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NOTE: This code may not work if PLY has different structure of header
    %% save PLY file
    fid = fopen(filename_save, 'w');
        for i = 1:size(HEADER, 1)
            fprintf(fid, '%s', HEADER{i, 1});
        end
        for i = 1:size(ListVertex, 1)
            if size(ListVertex, 2) == 3
                fprintf(fid, '%f %f %f\n', ListVertex(i, 1), ListVertex(i, 2), ListVertex(i, 3));
            elseif size(ListVertex, 2) == 6
                fprintf(fid, '%f %f %f %d %d %d\n', ListVertex(i, 1), ListVertex(i, 2), ListVertex(i, 3), ListVertex(i, 4), ListVertex(i, 5), ListVertex(i, 6));
            end
        end
        for i = 1:size(ListFace_backup, 1)
            fprintf(fid, '%d %d %d %d\n', ListFace_backup(i, 1), ListFace_backup(i, 2), ListFace_backup(i, 3), ListFace_backup(i, 4));
        end
    fclose(fid);