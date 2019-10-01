function [ListVertex, ListFace, ListFace_backup, HEADER] = function_loading_ply_file(filename)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab Tutorial for 3D Anthropometry
    %
    % Wonsup Lee
    % 07 Sep 2015
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NOTE: This code may not work if PLY has different structure of header

    %% read ply
    fid = fopen(filename, 'r'); % open a STL file
        % search number of HEADER lines
        for i = 1:100
            temp = fgets(fid);
            if strncmp(temp, 'end_header', 10) % stop if 'end-header' is found
                no_HEADER_lines = i;
                break;
            end
        end

        fseek(fid, 0, 'bof'); % go back to the first line
        % get HEADER
        for i = 1:no_HEADER_lines
            HEADER{i, 1} = fgets(fid);
            % search number of vertex
            if strncmp(HEADER{i, 1}, 'element vertex', 14)
                no_Vertex = strsplit(HEADER{i, 1});
                no_Vertex = str2double(no_Vertex(3));
            end
            % search number of face
            if strncmp(HEADER{i, 1}, 'element face', 12)
                no_Face = strsplit(HEADER{i, 1});
                no_Face = str2double(no_Face(3));
            end
        end

       % search number of numbers per vertex (e.g., 3 numbers with non-colored ply data. 6 numbers with colored ply data.)
        temp = fgets(fid);
        temp = strsplit(temp);
        temp = str2double(temp);
        temp(:, isnan(temp(1, :))) = [];
        % get vertex list
        if size(temp, 2) == 3 % if there are 3 numbers per line
            ListVertex = fscanf(fid, '%g %g %g', [3, no_Vertex-1]);
            ListVertex = ListVertex';
            ListVertex = [temp; ListVertex];
        elseif size(temp, 2) == 6 % if there are 6 numbers per line
            ListVertex = fscanf(fid, '%g %g %g %d %d %d', [6, no_Vertex-1]);
            ListVertex = ListVertex';
            ListVertex = [temp; ListVertex];
        end

        % get face list
            ListFace = fscanf(fid, '%g %g %g %g', [4, no_Face]);
            ListFace = ListFace';
            ListFace_backup = ListFace;
            ListFace(:, 1) = []; % delete the first column
            ListFace(:, :) = ListFace(:, :) + 1;
    fclose(fid); % opened file should be closed
    
    if size(ListVertex, 2) > 3
        fprintf('WARNING: Number of columns of the Vertex matrix = %d. Texture data may be included in the vertex matrix.\n', size(ListVertex, 2));
    end