function LMs = function_get_LM_from_iges(filename)
    fid = fopen(filename);
        clear tmp
        fseek(fid, 0, 'eof');
        fileSize = ftell(fid);
        frewind(fid);
        tmp = fread(fid, fileSize, 'uint8');
        numLines = sum(tmp == 10);
    fclose(fid);
    
    fid = fopen(filename);
        cnt = 1;
        for i = 1:numLines
            clear tmp
            tmp = fgets(fid);
            tmp = strsplit(tmp, {',', ';'});
            if str2double(tmp{1}) == 116
                LMs(cnt, 1) = str2double(tmp{2});
                LMs(cnt, 2) = str2double(tmp{3});
                LMs(cnt, 3) = str2double(tmp{4});
                cnt = cnt + 1;
            end
        end
    fclose(fid);
    assert(~isempty(LMs), sprintf('There are no landmarks saved in IGES file. Filename: %s', filename))