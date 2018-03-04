%Estimate optical flow using iterative method (image brightness change constancy)
%Inputs
    %imageSetRGB - set of input images
    %numImages - number of input images
	%stateCurrent - feature positions where optical flow needs to be estimated
    %OFWinSizeSpatial - window size for constancy assumption
	%OFWinSizeTemporal - temporal window size for constancy assumption
    %thresCondition - threshold for max value of the condition of the matrix
    %thresMin - threshold for min value of the flow vector magnitude
	%thresStopSSD - threshold for difference of SSD scores
	%thresStopIters - threshold for number of iterations of flow estimation
%Ouputs
    %stateNext - feature positions in the next image
function stateNext = iterativeOpticalFlow_2(imageSetRGB, numImages, stateCurrent, OFWinSizeSpatial, OFWinSizeTemporal, thresCondition, thresMin, thresStopSSD, thresStopIters)
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
[m,n,ch] = size(imageSet(:,:,1));
midImageIndex = int32((numImages + 1)/2);%equivalent to ceil
OFWinSizeSpatial_1 = int32(OFWinSizeSpatial - 1);
OFWinSizeSpatial_2 = int32(OFWinSizeSpatial_1/2);
OFWinSizeTemporal_1 = int32(OFWinSizeTemporal - 1);
OFWinSizeTemporal_2 = int32(OFWinSizeTemporal_1/2);

%Smoothen the image first
imageSetSmoothed = zeros(size(imageSet));
for i = 1:numImages
    imageSetSmoothed(:,:,i) = imgaussfilt(imageSet(:,:,i));
end
%Compute gradients using differences
%Gradients at t
imageTranslateXplus_t = imtranslate(imageSetSmoothed(:,:,midImageIndex),[-1 0]);%It(x+1)
imageTranslateYplus_t = imtranslate(imageSetSmoothed(:,:,midImageIndex),[0 -1]);%It(y+1)
imageTranslateXminus_t = imtranslate(imageSetSmoothed(:,:,midImageIndex),[1 0]);%It(x - 1)
imageTranslateYminus_t = imtranslate(imageSetSmoothed(:,:,midImageIndex),[0 1]);%It(y - 1)

imageTranslateXplus_t(:,end) = imageTranslateXplus_t(:,end - 1);
imageTranslateYplus_t(end,:) = imageTranslateYplus_t(end - 1,:);
imageTranslateXminus_t(:,1) = imageTranslateXplus_t(:,2);
imageTranslateYminus_t(1,:) = imageTranslateYplus_t(2,:);

gx_t = imageTranslateXplus_t - imageSetSmoothed(:,:,midImageIndex);
gy_t = imageTranslateYplus_t - imageSetSmoothed(:,:,midImageIndex);

%Gradients at t+1
imageTranslateXplus_t1 = imtranslate(imageSetSmoothed(:,:,midImageIndex + 1),[-1 0]);%Itp1(x+1)
imageTranslateYplus_t1 = imtranslate(imageSetSmoothed(:,:,midImageIndex + 1),[0 -1]);
imageTranslateXplus_t1(:,end) = imageTranslateXplus_t1(:,end - 1);
imageTranslateYplus_t1(end,:) = imageTranslateYplus_t1(end - 1,:);

gx_t1 = imageTranslateXplus_t1 - imageSetSmoothed(:,:,midImageIndex + 1);
gy_t1 = imageTranslateYplus_t1 - imageSetSmoothed(:,:,midImageIndex + 1);

gt = imageSetSmoothed(:,:,midImageIndex + 1) - imageSetSmoothed(:,:,midImageIndex);

gxTranslateY = imtranslate(gx_t,[0 -1]);

gxx = imageTranslateXplus_t + imageTranslateXminus_t - 2*imageSetSmoothed(:,:,midImageIndex);
gxy = gxTranslateY - gx_t;
gyy = imageTranslateYplus_t + imageTranslateYminus_t - 2*imageSetSmoothed(:,:,midImageIndex);
gxt = gx_t1 - gx_t;
gyt = gy_t1 - gy_t;
gtt = imageSetSmoothed(:,:,midImageIndex + 1) + imageSetSmoothed(:,:,midImageIndex - 1) - 2*imageSetSmoothed(:,:,midImageIndex);
zeroFlag = 0;
for i = 1:length(vctX)
    x = vctX(i);
    y = vctY(i);
    [gridX, gridY] = meshgrid(x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2, y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2);
    
    matA = [interp2(gx_t,x,y) interp2(gy_t,x,y) -1; ...
            interp2(gxx,x,y) interp2(gxy,x,y) 0; ...
            interp2(gxy,x,y) interp2(gyy,x,y) 0; ...
            interp2(gxt,x,y) interp2(gyt,x,y) 0];
    matB = -[interp2(gt,x,y) interp2(gxt,x,y) interp2(gyt,x,y) interp2(gtt,x,y)]';
    
    %Construct matrix A, B for linear system
%{
    matA = [gx_t(y,x) gy_t(y,x) -1; ...
            gxx(y,x) gxy(y,x) 0; ...
            gxy(y,x) gyy(y,x) 0; ...
            gxt(y,x) gyt(y,x) 0];
    matB = -[gt(y,x) gxt(y,x) gyt(y,x) gtt(y,x)]';
%}
    scoreSSD = sum(matB.^2);
    %Check condition number of A
    cond_matA = cond(matA);
    if(cond_matA > thresCondition)
        flow = [0 0 0]';
        zeroFlag = 1;
    else
        flow = inv(matA'*matA)*matA'*matB;
        if(norm(flow(1:2)) < thresMin)
            flow = [0 0 flow(3)]';
            zeroFlag = 1;
        end
    end
    
    iter = 0;
    u = flow(1); v = flow(2);
    netFlow = [u v];
    diffScoreSSD = realmax;
    while((iter < thresStopIters) && (diffScoreSSD > thresStopSSD) && (zeroFlag == 0) ...
            && (x-OFWinSizeSpatial_2+u >= 1) && (x+OFWinSizeSpatial_2+u <= n) ...
            && (y-OFWinSizeSpatial_2+v >= 1) && (y+OFWinSizeSpatial_2+v <= m))
        imageShifted = interp2(imageSetSmoothed(:,:,midImageIndex + 1), double(gridX) + u, double(gridY) + v);
        
        gx_imageShifted = imtranslate(imageShifted,[-1 0]) - imageShifted;
        gxt_temp = gx_imageShifted - interp2(gx_t,double(gridX), double(gridY));
        gy_imageShifted = imtranslate(imageShifted,[0 -1]) - imageShifted;
        gyt_temp = gy_imageShifted - interp2(gy_t,double(gridX), double(gridY));
        
        gt_temp = imageShifted - interp2(imageSetSmoothed(:,:,midImageIndex),double(gridX), double(gridY));
%         imageSetSmoothed(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2,x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2,midImageIndex);
        gtt_temp = gt_temp + interp2(imageSetSmoothed(:,:,midImageIndex - 1),double(gridX), double(gridY)) ...
                            - interp2(imageSetSmoothed(:,:,midImageIndex),double(gridX), double(gridY));
%         imageSetSmoothed(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2,x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2,midImageIndex - 1)...
%                     - imageSetSmoothed(y-OFWinSizeSpatial_2:y+OFWinSizeSpatial_2,x-OFWinSizeSpatial_2:x+OFWinSizeSpatial_2,midImageIndex);
        
        matA = [interp2(gx_t,x,y) interp2(gy_t,x,y) -1; ...
                interp2(gxx,x,y) interp2(gxy,x,y) 0; ...
                interp2(gxy,x,y) interp2(gyy,x,y) 0; ...
                gxt_temp(3,3) gyt_temp(3,3) 0];
        matB = -[gt_temp(3,3) gxt_temp(3,3) gyt_temp(3,3) gtt_temp(3,3)]';
        
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