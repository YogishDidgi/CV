%Function to get the next state of the feature points
%Inputs
    %stateCurrent - state vector in the current frame
    %imageSet - set of input images
	%flowMethod - method used for tracking
	%kalmanParams - covariance matrix for each feature point needed for recursion
%Ouputs
    %stateNext - state vector in the next frame
	%kalmanParams - covariance matrix for each feature point needed for recursion
function [stateNext, kalmanParams] = getNextState(stateCurrent, imageSet, flowMethod, kalmanParams)
numImages = size(imageSet,4);
midImageIndex = uint32(ceil(numImages/2));
numFeatures = size(stateCurrent, 1);
stateNext = zeros(size(stateCurrent));

if(strcmp(flowMethod,'OF1'))
    %Initialization parameters
    parameters = getConfigParams(1);
    %Call OF1 method and get position from optical flow vectors (just add? vector addition;)
%     stateNext = calcOpticalFlow_1(imageSet, numImages, stateCurrent, parameters.OFWinSizeSpatial, parameters.thresCondition, parameters.thresMin);
    stateNext = iterativeOpticalFlow_1(imageSet, numImages, stateCurrent, parameters.OFWinSizeSpatial, ...
                                    parameters.thresCondition, parameters.thresMin, parameters.thresStopSSD, parameters.thresStopIters);
elseif(strcmp(flowMethod,'OF2'))
    %Initialization parameters
    parameters = getConfigParams(1);
    %Call OF2 method and get position from optical flow vectors (just add? vector addition;)
%     stateNext = calcOpticalFlow_2(imageSet, numImages, stateCurrent, parameters.OFWinSizeSpatial, ...
%                                     parameters.OFWinSizeTemporal, parameters.thresCondition, parameters.thresMin);
	stateNext = iterativeOpticalFlow_2(imageSet, numImages, stateCurrent, parameters.OFWinSizeSpatial, ...
                                    parameters.OFWinSizeTemporal, parameters.thresCondition, parameters.thresMin, ...
                                    parameters.thresStopSSD, parameters.thresStopIters);
else
    %Initialization parameters
    parameters = getConfigParams(2);
    
    %Call Kalman method
    stateNextDummy = stateCurrent;
    if(sum(sum(kalmanParams)) ~= 0)
        sigmaNextDummy = kalmanParams;
    else
        sigmaNextDummy = repmat(parameters.sigma_init,[1 1 numFeatures]);
    end
    kalmanParamsDummy = zeros(size(parameters.sigma_init,1),size(parameters.sigma_init,2),numFeatures);
    %Multiple kalman; One for each feature point
	for iter3 = 1:numFeatures
        [stateNextDummy2, sigmaNextDummy2] = KalmanFiltering(imageSet(:,:,:,midImageIndex - 1), imageSet(:,:,:,midImageIndex), stateNextDummy(iter3,:)', parameters, sigmaNextDummy(:,:,iter3));
        stateNext(iter3,:) = stateNextDummy2';
        kalmanParamsDummy(:,:,iter3) = sigmaNextDummy2;
    end
    kalmanParams = kalmanParamsDummy;
end

end