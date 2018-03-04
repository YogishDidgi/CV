%Function for tracking the feature across images using kalman filter
%Inputs
	%imageCurrent - current image
	%imageNext - next image
	%stateCurrent - state in current image
	%parameters - kalman filter configuration parameters
	%sigmaCurrent - covariance estimate of current state
%Outputs
	%stateNext - state in next image
	%sigmaNext - covariance estimate of next state
function [stateNext, sigmaNext] = KalmanFiltering(imageCurrent, imageNext, stateCurrent, parameters, sigmaCurrent)
STM = parameters.STM;
H = parameters.H;
Q = parameters.Q;
R = parameters.R;

searchWidth = parameters.searchWidth;
searchHeight = parameters.searchHeight;
windowSizeScale = parameters.windowSizeScale;
colorProcessing = parameters.colorProcessing;
searchSize = [searchWidth, searchHeight]';

stateNextEstimate = computeStateEstimate(stateCurrent, STM);
sigmaNextEstimate = computeSigmaEstimate(sigmaCurrent, STM, Q);
kalmanGain = computeKalmanGain(sigmaNextEstimate, H, R);
stateMeasurement = getMeasurement(imageCurrent, imageNext, stateCurrent, stateNextEstimate, sigmaNextEstimate, searchSize, windowSizeScale, colorProcessing);
stateNext = updateState(stateNextEstimate, stateMeasurement, kalmanGain, H);
sigmaNext = updateSigma(sigmaNextEstimate, kalmanGain, H);
end

%estimate next state of the system
%Inputs
    %stateCurrent - state in current image
    %STM - state transition matrix
%Ouputs
    %stateNextEstimate - estimate of the next state
function stateNextEstimate = computeStateEstimate(stateCurrent, STM)
stateNextEstimate = STM*stateCurrent;
end

%compute estimate of the covariance
%Inputs
    %sigmaCurrent - covariance estimate of current state
    %STM - state transition matrix
    %Q - covariance of noise in covariance estimate
%Ouputs
    %sigmaNextEstimate - covariance estimate of next state
function sigmaNextEstimate = computeSigmaEstimate(sigmaCurrent, STM, Q)
sigmaNextEstimate = STM*sigmaCurrent*STM' + Q;
end

%compute kalman gain of the system
%Inputs
    %sigmaNextEstimate - covariance estimate of next state
    %H - measurement matrix
    %R - covariance of noise in measurement
%Ouputs
    %kalmanGain - computed kalman gain
function kalmanGain = computeKalmanGain(sigmaNextEstimate, H, R)
tempMat_1 = sigmaNextEstimate*H';
tempMat_2 = H*tempMat_1 + R;
kalmanGain = tempMat_1*inv(tempMat_2);
end

%measure the next state of the system
%Inputs
    %inputImageCurrent - current image
    %inputImageNext - next image
    %stateCurrent - state in current image
    %stateNextEstimate - estimate of the next state
    %sigmaNextEstimate - covariance estimate of next state
    %faceSize - face bounding box
    %windowSizeThreshold - scale factor for search window size
    %colorProcessing - flag to decide which image to work on HSV/grayscale
%Ouputs
    %stateMeasurement - measurement of next state
function stateMeasurement = getMeasurement(inputImageCurrent, inputImageNext, stateCurrent, stateNextEstimate, sigmaNextEstimate, faceSize, windowSizeThreshold, colorProcessing)
[imageHeight, imageWidth, imageChannels] = size(inputImageCurrent);
%Determine window size
eigenValues = eig(sigmaNextEstimate(1:2,1:2));
windowSizeX = int32(windowSizeThreshold*sqrt(eigenValues(1)));
windowSizeY = int32(windowSizeThreshold*sqrt(eigenValues(2)));

%Get face region in current frame
faceSizeWidth_1 = faceSize(1)/2;
faceSizeHeight_1 = faceSize(2)/2;
xMin = int32(stateCurrent(1) - faceSizeWidth_1);
xMax = int32(xMin + faceSize(1) - 1);
yMin = int32(stateCurrent(2) - faceSizeHeight_1);
yMax = int32(yMin + faceSize(2) - 1);

if(colorProcessing)
    inputImageCurrent = rgb2hsv(inputImageCurrent);
    inputImageNext = rgb2hsv(inputImageNext);
    tempMatCurrent = double(inputImageCurrent(yMin:yMax, xMin:xMax,:));
    tempMatMask = ones(size(tempMatCurrent));
%     tempMatMask(faceSizeHeight_1-9:faceSizeHeight_1+9,faceSizeWidth_1-9:faceSizeWidth_1+9,:) = 0;
else
    %Grayscale images
    if(size(inputImageCurrent,3) ~= 1)
        inputImageCurrentGray = double(rgb2gray(inputImageCurrent));
        inputImageNextGray = double(rgb2gray(inputImageNext));
    else
        inputImageCurrentGray = double(inputImageCurrent);
        inputImageNextGray = double(inputImageNext);
    end
    tempMatCurrent = inputImageCurrentGray(yMin:yMax, xMin:xMax);
    tempMatMask = ones(size(tempMatCurrent));
%     tempMatMask(faceSizeHeight_1-9:faceSizeHeight_1+9,faceSizeWidth_1-9:faceSizeWidth_1+9) = 0;
end
tempMatCurrent = tempMatCurrent.*tempMatMask;

vectorSSD = zeros(windowSizeX*windowSizeY,1);

%Get the search window dimensions
windowSizeX_1 = windowSizeX/2;
windowSizeY_1 = windowSizeY/2;
xMin = stateNextEstimate(1) - windowSizeX_1;
xMax = xMin + windowSizeX - 1;
yMin = stateNextEstimate(2) - windowSizeY_1;
yMax = yMin + windowSizeY - 1;
index = 1;

minValLoop = realmax;
minIndexLoopX = 0;
minIndexLoopY = 0;

%Iterate through all pixels in search window
for y = yMin:yMax
    for x = xMin:xMax
        yMinTemp = y - faceSizeHeight_1;
        yMaxTemp = yMinTemp + faceSize(2) - 1;
        xMinTemp = x - faceSizeWidth_1;
        xMaxTemp = xMinTemp + faceSize(1) - 1;

        %Handle border cases
        xMinChange = 1; xMaxChange = 0; yMinChange = 1; yMaxChange = 0;
        if (xMinTemp < 1)
            xMaxChange = 1 - xMinTemp;
            xMinTemp = 1;
        end
        if (xMaxTemp > imageWidth)
            xMinChange = xMaxTemp - imageWidth + 1;
            xMaxTemp = imageWidth;
        end
        if (yMinTemp < 1)
            yMaxChange = 1 - yMinTemp;
            yMinTemp = 1;
        end
        if (yMaxTemp > imageHeight)
            yMinChange = yMaxTemp - imageHeight + 1;
            yMaxTemp = imageHeight;
        end
        if(colorProcessing)
            tempMatNext = zeros(faceSize(2),faceSize(1),imageChannels);
            tempMatNext(yMinChange:(faceSize(2) - yMaxChange), xMinChange:(faceSize(1) - xMaxChange),:) ...
                        = double(inputImageNext(yMinTemp:yMaxTemp, xMinTemp:xMaxTemp,:));
            tempMatNext = tempMatNext.*tempMatMask;
            vectorSSD(index) = (sum(sum(sum((tempMatCurrent - tempMatNext).*(tempMatCurrent - tempMatNext)))));%perform SSD
        else
            tempMatNext = zeros(faceSize(2),faceSize(1));
            tempMatNext(yMinChange:(faceSize(2) - yMaxChange), xMinChange:(faceSize(1) - xMaxChange)) ...
                        = inputImageNextGray(yMinTemp:yMaxTemp, xMinTemp:xMaxTemp);
            tempMatNext = tempMatNext.*tempMatMask;
            vectorSSD(index) = (sum(sum((tempMatCurrent - tempMatNext).*(tempMatCurrent - tempMatNext))));%perform SSD
        end
        %Store the best candidate
        if (vectorSSD(index) < minValLoop)
            minValLoop = vectorSSD(index);
            minIndexLoopX = x;
            minIndexLoopY = y;
        end
        index = index + 1;
    end
end
stateMeasurement = double([minIndexLoopX minIndexLoopY]');
end

%update the state of the system
%Inputs
    %stateNextEstimate - estimate of the next state
    %stateMeasurement - measurement of next state
    %kalmanGain - kalman gain factor
    %H - measurement matrix
%Ouputs
    %stateNext - updated next state
function stateNext = updateState(stateNextEstimate, stateMeasurement, kalmanGain, H)
stateNext = stateNextEstimate + kalmanGain*(stateMeasurement - H*stateNextEstimate);
end

%update the covariance matrix
%Inputs
    %sigmaNextEstimate - estimate of the covariance matrix
    %kalmanGain - kalman gain factor
    %H - measurement matrix
%Ouputs
    %sigmaNext - updated covariance matrix
function sigmaNext = updateSigma(sigmaNextEstimate, kalmanGain, H)
tempMat_1 = kalmanGain*H;
sigmaNext = (eye(size(tempMat_1,1)) - tempMat_1)*sigmaNextEstimate;
end