%Estimate optical flow using Lucas-Kanade method (image brightness constancy)
%Inputs
    %imageSet - set of input images
    %numImages - number of input images
	%stateCurrent - feature positions where optical flow needs to be estimated
    %OFWinSizeSpatial - window size for constancy assumption
    %thresCondition - threshold for max value of the condition of the matrix
    %thresMin - threshold for min value of the flow vector magnitude
%Ouputs
    %stateNext - feature positions in the next image
function stateNext = calcOpticalFlow_1(imageSetRGB, numImages, stateCurrent, OFWinSizeSpatial, thresCondition, thresMin)
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
[m,n] = size(imageSet(:,:,1));
flowImage = zeros(m, n, 2);

%Smoothen the image first
imageSetSmoothed = zeros(size(imageSet));
for i = 1:numImages
    imageSetSmoothed(:,:,i) = imgaussfilt(imageSet(:,:,i));
end

%Compute gradients using differences
% [gx,gy] = gradient(imageSetSmoothed(:,:,midImageIndex));
imageTranslateX = imtranslate(imageSetSmoothed(:,:,midImageIndex),[-1 0]);
imageTranslateY = imtranslate(imageSetSmoothed(:,:,midImageIndex),[0 -1]);
imageTranslateX(:,end) = imageTranslateX(:,end - 1);
imageTranslateY(end,:) = imageTranslateY(end - 1,:);

gx = imageTranslateX - imageSetSmoothed(:,:,midImageIndex);
gy = imageTranslateY - imageSetSmoothed(:,:,midImageIndex);
gt = imageSetSmoothed(:,:,midImageIndex + 1) - imageSetSmoothed(:,:,midImageIndex);

for i = 1:length(vctX)
    x = vctX(i);
    y = vctY(i);
    
    matA = [reshape(gx(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1) ...
            reshape(gy(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1)];
	matB = -reshape(gt(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1);
    %Check condition number of A
    cond_matA = cond(matA);
    if(cond_matA > thresCondition)
        flow = [0 0]';
    else
        flow = inv(matA'*matA)*matA'*matB;
        if(norm(flow) < thresMin)
            flow = [0 0]';
        end
    end
    flowVct(i,:) = flow;
end

%get position from optical flow vectors
stateNext = stateCurrent + flowVct;
end