%Function for performing non-rigid factorization for 3D reconstruction
%Inputs
	%matPosition - matrix containing the 2D points of all feature points in all the frames
%Outputs
	%points3D - matrix containing the estimated 3D points of all feature points in all the frames
function points3D = perform3DReconstructionNonRigid(matPosition)
[numImagesX2,numPoints] = size(matPosition);
numImages = uint32(numImagesX2/2);

%Parameter
numBasis = 16;%16
if((3*numBasis) > min(numImagesX2,numPoints))
    numBasis = floor(min(numImagesX2,numPoints)/3);
end

%centroid subtraction
matCentroid = sum(matPosition, 2)/numPoints;
matPositionRelative = matPosition - repmat(matCentroid,1,numPoints);
points3D = zeros(numImages*3,numPoints);

%factorization
[U1,D1,V1] = svd(matPositionRelative);

%Impose rank=3*k constraint
% D1(4:end,4:end) = 0;
U2 = U1(:,1:3*numBasis);
D2 = D1(1:3*numBasis,1:3*numBasis);
V2 = V1(:,1:3*numBasis);

Q = U2*sqrt(D2);
B = sqrt(D2)*V2';

L = zeros(numImages, numBasis);
R1 = zeros(numImagesX2, 3);

for i = 1:numImages%(size(Q,1)/2)
    q = Q(2*i - 1:2*i,:);
    q_bar = reorder(q, numBasis);
    [U,D,V] = svd(q_bar);
    %Impose rank=1 constraint
    L(i,:) = U(:,1)*sqrt(D(1));
    R1(2*i - 1:2*i,:) = reshape(sqrt(D(1))*V(:,1)',3,2)';
end

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

X = (inv(A'*A))*A'*b;
matX = [X(1) X(2) X(3);...
        X(2) X(4) X(5);...
        X(3) X(5) X(6);];
[Q2,PD_flag] = chol(matX,'lower');
if(PD_flag)%Check for positive definiteness
    [vec,val] = eig(matX);
    val(val<0) = eps;
    Q3 = vec*val*vec';
    for i = 1:numBasis
        basis(3*i-2:3*i,:) = inv(chol(Q3))*B(3*i-2:3*i,:);
    end
else
    for i = 1:numBasis
        basis(3*i-2:3*i,:) = inv(Q2)*B(3*i-2:3*i,:);
    end
end
%Get 3D points estimation
for i = 1:numImages
    for j = 1:numBasis
        points3D(3*i-2:3*i,:) = points3D(3*i-2:3*i,:) + L(i,j)*basis(3*j-2:3*j,:);
    end
end
end

function b = reorder(a, k)
b = zeros(k,6);
for i = 1:k
    b(i,:) = [a(1,3*i-2:3*i) a(2,3*i-2:3*i)];
end
end