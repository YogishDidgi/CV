%Function for performing rigid factorization for 3D reconstruction
%Inputs
	%matPosition - matrix containing the 2D points of all feature points in all the frames
%Outputs
	%points3D - matrix containing the estimated 3D points of all feature points
function points3D = perform3DReconstructionRigid(matPosition)

%centroid subtraction
[numImagesX2,numPoints] = size(matPosition);
numImages = uint32(numImagesX2/2);
matCentroid = sum(matPosition, 2)/numPoints;
matPositionRelative = matPosition - repmat(matCentroid,1,numPoints);

%factorization
[U1,D1,V1] = svd(matPositionRelative);

%Impose rank=3 constraint
D1(4:end,4:end) = 0;
U2 = U1(:,1:3);
D2 = D1(1:3,1:3);
V2 = V1(:,1:3);

R1 = U2*sqrt(D2);
S1 = sqrt(D2)*V2';

%Construct AX = b
A = zeros(3*numImages, 6);%Number of unknowns = 6;because of symmetry
b = zeros(3*numImages, 1);
for i = 1:numImages
    iter_A = 3*(i - 1);
    iter_R1 = 2*(i - 1);
    row1 = R1(iter_R1 + 1,:);%1x3
    row2 = R1(iter_R1 + 2,:);%1x3
    row11 = row1'*row1;%3x3
    row22 = row2'*row2;%3x3
    row12 = row1'*row2;%3x3
    row11temp = [row11(1) row11(2)+row11(4) row11(3)+row11(7) row11(5) row11(6)+row11(8) row11(9)]';
    row22temp = [row22(1) row22(2)+row22(4) row22(3)+row22(7) row22(5) row22(6)+row22(8) row22(9)]';
    row12temp = [row12(1) row12(2)+row12(4) row12(3)+row12(7) row12(5) row12(6)+row12(8) row12(9)]';
    %A(iter_A + 1:iter_A + 3,:) = [row11(:) row22(:) row12(:)]';
    A(iter_A + 1:iter_A + 3,:) = [row11temp row22temp row12temp]';
    b(iter_A + 1:iter_A + 3,:) = [1; 1; 0];
end
% X = A\b;
X = (inv(A'*A))*A'*b;
matX = [X(1) X(2) X(3);...
        X(2) X(4) X(5);...
        X(3) X(5) X(6);];
% Q1 = reshape(X,[3 3]);
[Q2,PD_flag] = chol(matX,'lower');
if(PD_flag)%Check for positive definiteness
    [vec,val] = eig(matX);
    val(val<0) = eps;
    Q3 = vec*val*vec';
    points3D = inv(chol(Q3))*S1;
else
    points3D = inv(Q2)*S1;
end
end