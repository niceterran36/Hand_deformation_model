addpath 'functions'
ListVertex = transformed.vertices;
ListFace_backup = transformed.faces;

% Header »ý¼º
% load('THD.mat'); 
THD = cell(10,1);
THD{1} = ['ply', newline()];
THD{2} = ['format ascii 1.0', newline()];
THD{3} = ['comment Exported by RapidForm', newline()];
THD{4} = ['element vertex 3045', newline()];
THD{5} = ['property float x', newline()];
THD{6} = ['property float y', newline()];
THD{7} = ['property float z', newline()];
THD{8} = ['element face 6036', newline()];
THD{9} = ['property list uchar int vertex_index', newline()];
THD{10} = ['end_header', newline()];

TF = transformed.faces;

TF2 = zeros(6036,3);
TF2(:,2:4) = TF;
TF2(:,1) = 3;



function_saving_ply_file(ListVertex, TF2, THD, 'default_template.ply')