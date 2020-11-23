function A_tr = segment_scale_simple(A, compare_factor)

% D2 scale
C1 = A(16,:); 
C2 = A(15,:);
C3 = A(14,:);
C4 = A(13,:);
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(12)*vector1;
vector2_n = compare_factor(11)*vector2;
vector3_n = compare_factor(10)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

A(15,:) = C2_n; 
A(14,:) = C3_n;
A(13,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1

% D3 scale
C1 = A(12,:); 
C2 = A(11,:); 
C3 = A(10,:); 
C4 = A(9,:); 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(9)*vector1;
vector2_n = compare_factor(8)*vector2;
vector3_n = compare_factor(7)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

A(11,:) = C2_n; 
A(10,:) = C3_n;
A(9,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1

% D4 scale
C1 = A(8,:);  
C2 = A(7,:);  
C3 = A(6,:);  
C4 = A(5,:);  
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(6)*vector1;
vector2_n = compare_factor(5)*vector2;
vector3_n = compare_factor(4)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

A(7,:) = C2_n; 
A(6,:) = C3_n;
A(5,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1

% D5 scale

C1 = A(4,:);
C2 = A(3,:);
C3 = A(2,:);
C4 = A(1,:); 
CP1 = [C1; C2; C3; C4];

vector1 = CP1(2,:) - CP1(1,:);
vector2 = CP1(3,:) - CP1(2,:);
vector3 = CP1(4,:) - CP1(3,:);

vector1_n = compare_factor(3)*vector1;
vector2_n = compare_factor(2)*vector2;
vector3_n = compare_factor(1)*vector3;

C2_n = CP1(1,:) + vector1_n;
C3_n = C2_n + vector2_n;
C4_n = C3_n + vector3_n;
CP2 = [CP1(1,:); C2_n; C3_n; C4_n];

A(3,:) = C2_n; 
A(2,:) = C3_n;
A(1,:) = C4_n;

clear vector1 vector2 vector3 vector1_n vector2_n vector3_n C2_n C3_n C4_n C1 C2 C3 C4 CP1

A_tr = A;

end 