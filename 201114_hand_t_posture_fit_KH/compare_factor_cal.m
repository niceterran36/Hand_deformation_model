function compare_factor = compare_factor_cal(A,B)

compare_link = zeros(12,2);
compare_link(1,1) = norm(A(1,:)-A(2,:)); % template
compare_link(1,2) = norm(B(1,:)-B(2,:)); % target scan
compare_link(2,1) = norm(A(2,:)-A(3,:)); 
compare_link(2,2) = norm(B(2,:)-B(3,:)); 
compare_link(3,1) = norm(A(3,:)-A(4,:)); 
compare_link(3,2) = norm(B(3,:)-A(4,:)); 
compare_link(4,1) = norm(A(5,:)-A(6,:)); 
compare_link(4,2) = norm(B(5,:)-B(6,:)); 
compare_link(5,1) = norm(A(6,:)-A(7,:)); 
compare_link(5,2) = norm(B(6,:)-B(7,:)); 
compare_link(6,1) = norm(A(7,:)-A(8,:)); 
compare_link(6,2) = norm(B(7,:)-A(8,:)); 
compare_link(7,1) = norm(A(9,:)-A(10,:)); 
compare_link(7,2) = norm(B(9,:)-B(10,:)); 
compare_link(8,1) = norm(A(10,:)-A(11,:)); 
compare_link(8,2) = norm(B(10,:)-B(11,:)); 
compare_link(9,1) = norm(A(11,:)-A(12,:)); 
compare_link(9,2) = norm(B(11,:)-A(12,:)); 
compare_link(10,1) = norm(A(13,:)-A(14,:)); 
compare_link(10,2) = norm(B(13,:)-B(14,:)); 
compare_link(11,1) = norm(A(14,:)-A(15,:)); 
compare_link(11,2) = norm(B(14,:)-B(15,:)); 
compare_link(12,1) = norm(A(15,:)-A(16,:)); 
compare_link(12,2) = norm(B(15,:)-A(16,:)); 
compare_factor = compare_link(:,2)./compare_link(:,1);

compare_factor(1) = 1;
compare_factor(4) = 1;
compare_factor(7) = 1;
compare_factor(10) = 1;
end 