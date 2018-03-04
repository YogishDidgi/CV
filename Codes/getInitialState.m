%Function to get the initial state of the feature points
%Inputs
    %videoName - name of the video being used
    %numFeatures - number of features tracked
	%flowMethod - method used for tracking
%Ouputs
    %stateCurrent - initial state vector
function stateCurrent = getInitialState(videoName, numFeatures, flowMethod)
data = importdata(sprintf('%s/data.txt',videoName));

if(strcmp(flowMethod,'Kalman'))
    stateCurrent = zeros(numFeatures, 4);
    %Columns 1,2 contain x,y position data for 2nd frame-needed for kalman
    stateCurrent(:,1:2) = data(numFeatures+1:2*numFeatures,:);
    %Columns 3,4 contain vx,vy velocity data for 2nd frame-needed for kalman
    stateCurrent(:,3:4) = data(numFeatures+1:2*numFeatures,:) - data(1:numFeatures,:);
else
    stateCurrent = zeros(numFeatures, 2);
    %Columns 1,2 contain x,y position data for 3rd frame-needed for optical flow
    stateCurrent(:,1:2) = data(2*numFeatures+1:end,:);
end
end