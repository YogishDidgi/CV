%Estimate optical flow using iterative Lucas-Kanade method (image brightness constancy)
%Inputs
    %imageSetRGB - set of input images
    %numImages - number of input images
	%stateCurrent - feature positions where optical flow needs to be estimated
    %OFWinSizeSpatial - window size for constancy assumption
    %thresCondition - threshold for max value of the condition of the matrix
    %thresMin - threshold for min value of the flow vector magnitude
	%thresStopSSD - threshold for difference of SSD scores
	%thresStopIters - threshold for number of iterations of flow estimation
%Ouputs
    %stateNext - feature positions in the next image
function stateNext = iterativeOpticalFlow_1(imageSetRGB, numImages, stateCurrent, OFWinSizeSpatial, thresCondition, thresMin, thresStopSSD, thresStopIters)
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
    [gridX, gridY] = meshgrid(x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2, y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2);
    matA1 = interp2(gx, double(gridX), double(gridY));
    matA2 = interp2(gy, double(gridX), double(gridY));
    matA = [reshape(matA1,[],1) reshape(matA2,[],1)];
    matBTemp = interp2(gt, double(gridX), double(gridY));
    matB = -reshape(matBTemp,[],1);
%     matA = [reshape(gx(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1) ...
%             reshape(gy(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1)];
% 	matB = -reshape(gt(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2),[],1);
    
    scoreSSD = sum(matB.^2);
    %Check condition number of A
    cond_matA = cond(matA);
    zeroFlag = 0;
    if(cond_matA > thresCondition)
        flow = [0 0]';
        zeroFlag = 1;
    else
        flow = inv(matA'*matA)*matA'*matB;
        if(norm(flow) < thresMin)
            flow = [0 0]';
            zeroFlag = 1;
        end
    end
    
    iter = 0;
    u = flow(1); v = flow(2);
    netFlow = [u v];
    diffScoreSSD = realmax;

    while((iter < thresStopIters) && (diffScoreSSD > thresStopSSD) && (zeroFlag == 0))
        imageShifted = interp2(imageSetSmoothed(:,:,midImageIndex + 1), double(gridX) + u, double(gridY) + v);
        gtNew = imageShifted - imageSetSmoothed(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2, x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2, midImageIndex);
        matB = -reshape(gtNew,[],1);
        newScoreSSD = sum(matB.^2);
        
        flow = inv(matA'*matA)*matA'*matB;
        if(norm(flow) < thresMin)
            flow = [0 0]';
            zeroFlag = 1;
        end
        u = flow(1);
        v = flow(2);
        diffScoreSSD = abs(scoreSSD - newScoreSSD);
        if(newScoreSSD > scoreSSD)
            break;
        end
        scoreSSD = newScoreSSD;
        netFlow = netFlow + [u v];
        iter = iter + 1;
    end
%     if(zeroFlag)
%         break;
%     end
    flowVct(i,:) = netFlow;
end

%get position from optical flow vectors
stateNext = stateCurrent(:,1:2) + flowVct;
end