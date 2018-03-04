%Function for returning the configuration parameters
%Inputs
	%methodType - flag determining which approach is used for feature tracking
%Outputs
	%parameters - configuration parameters
function parameters = getConfigParams(methodType)
if(methodType == 1)
    parameters = getOpticalFlowConfigParams();
else
    parameters = getKalmanConfigParams();
end
end

%Function for returning the optical flow configuration parameters
%Inputs
	%
%Outputs
	%parameters - optical flow configuration parameters
function parameters = getOpticalFlowConfigParams()
%Window size
f1 = 'OFWinSizeSpatial';v1 = 9;
f2 = 'OFWinSizeTemporal';v2 = 5;
%Thresholds
f3 = 'thresCondition';v3 = 1e3;
f4 = 'thresMin';v4 = 1e-4;
f5 = 'thresStopSSD';v5 = 0.1;
f6 = 'thresStopIters';v6 = 40;
parameters = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6);
end

%Function for returning the kalman filter configuration parameters
%Inputs
	%
%Outputs
	%parameters - kalman filter configuration parameters
function parameters = getKalmanConfigParams()
%For SSD
f1 = 'searchWidth';v1 = 10;
f2 = 'searchHeight';v2 = 10;

f3 = 'windowSizeScale';v3 = 5;
f4 = 'colorProcessing';v4 = 0;

f5 = 'STM';v5 = eye(4);v5(1,3) = 1;v5(2,4) = 1;
f6 = 'H';v6 = zeros(2,4);v6(1,1) = 1;v6(2,2) = 1;
f7 = 'Q';v7 = 4*eye(4);v7(1,1) = 16;v7(2,2) = 16;
f8 = 'R';v8 = 4*eye(2);
f9 = 'sigma_init';v9 = 25*eye(4);v9(1,1) = 100;v9(2,2) = 100;

parameters = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8,f9,v9);
end