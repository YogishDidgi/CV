%Estimate optical flow using Lucas-Kanade method (image brightness constancy)
%Inputs
    %imageSet - set of input images
    %numImages - number of input images
	%stateCurrent - feature positions where optical flow needs to be estimated
    %OFWinSizeSpatial - window size for constancy assumption
	%OFWinSizeTemporal - temporal window size for constancy assumption
    %thresCondition - threshold for max value of the condition of the matrix
    %thresMin - threshold for min value of the flow vector magnitude
%Ouputs
    %stateNext - feature positions in the next image
function stateNext = calcOpticalFlow_2(imageSetRGB, numImages, stateCurrent, OFWinSizeSpatial, OFWinSizeTemporal, thresCondition, thresMin)
vctX = stateCurrent(:,1);
vctY = stateCurrent(:,2);
flowVct = zeros(size(vctX,1),2);

for i = 1:numImages
    if(size(imageSetRGB,3) ~= 1)
        imageSet(:,:,i) = rgb2gray(imageSetRGB(:,:,:,i));
    else
        imageSet(:,:,i) = (imageSetRGB(:,:,:,i));
    end
end

midImageIndex = int32((numImages + 1)/2);%equivalent to ceil
OFWinSizeSpatial_1 = int32(OFWinSizeSpatial - 1);
OFWinSizeSpatial_2 = int32(OFWinSizeSpatial_1/2);
OFWinSizeTemporal_1 = int32(OFWinSizeTemporal - 1);
OFWinSizeTemporal_2 = int32(OFWinSizeTemporal_1/2);

%Construct D matrix
matD = zeros(OFWinSizeSpatial*OFWinSizeSpatial*OFWinSizeTemporal, 20);
index = 1;
for t = -OFWinSizeTemporal_2:1:OFWinSizeTemporal_2
    for x = -OFWinSizeSpatial_2:1:OFWinSizeSpatial_2
        for y = -OFWinSizeSpatial_2:1:OFWinSizeSpatial_2
            matD(index,:) = [1,x,y,t,x*x,x*y,y*y,y*t,t*t,x*t,x^3,y*x^2,x*y^2,y^3,t*y^2,y*t^2,t^3,t*x^2,x*t^2,x*y*t];
            index = index + 1;
        end
    end
end
matD_pinv = inv(matD'*matD)*matD';%pseudo-inverse of D

for i = 1:length(vctX)
    x = vctX(i);
    y = vctY(i);
    
    matJ = reshape(imageSet(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2, ...
                    midImageIndex-OFWinSizeTemporal_2:midImageIndex+OFWinSizeTemporal_2),[],1);
    a = matD_pinv*double(matJ);%Get the model coefficients

    %Construct matrix A, B for linear system
    matA = [a(2) a(3); 2*a(5) a(6); a(6) 2*a(7); a(10) a(8)];
    matB = -[a(4) a(10) a(8) 2*a(9)]';
    matA = [a(2) a(3) -1; 2*a(5) a(6) 0; a(6) 2*a(7) 0; a(10) a(8) 0];
	matB = -[a(4) a(10) a(8) 2*a(9)]';
        
    %Check condition number of A
    cond_matA = cond(matA);
    if(cond_matA > thresCondition)
        flow = [0 0 0]';
    else
        flow = inv(matA'*matA)*matA'*matB;
        if(norm(flow(1:2)) < thresMin)
            flow = [0 0 flow(3)]';
        end
    end
    flowVct(i,:) = flow(1:2);
end

%get position from optical flow vectors
stateNext = stateCurrent + flowVct;
end