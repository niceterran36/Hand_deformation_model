function data = function_rotation_matrix(data, R)
    data_copy = data;
    data(:, 1) = R(1, 1)*data_copy(:, 1) + R(1, 2)*data_copy(:, 2) + R(1, 3)*data_copy(:, 3);
    data(:, 2) = R(2, 1)*data_copy(:, 1) + R(2, 2)*data_copy(:, 2) + R(2, 3)*data_copy(:, 3);
    data(:, 3) = R(3, 1)*data_copy(:, 1) + R(3, 2)*data_copy(:, 2) + R(3, 3)*data_copy(:, 3);