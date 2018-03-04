%Main file
function FinalProject
close all
clear all
flowMethods = {'OF1','OF2','Kalman'};
reconstructionMethods = {'Rigid','Non-rigid'};
flowMethodUsed = 1;
reconstructionMethodUsed = 1;
%Get data
videoName = {'./Videos/Cube_Short','./Videos/Hotel','./Videos/franck_images-0999/images','./Videos/boxofjoe','./Videos/paper','./Videos/gyan'};
imageName = {'','hotel.seq','franck_','','',''};
imageEnd = {'.png','.png','.jpg'};
numFrames = [7,15,20,20,20,20];
numFeatures = [7,12,68,13,7,13];
ID = 6;%2-6

outVideoName = sprintf('%s/output_%d.avi',videoName{ID},flowMethodUsed);
outputVideoObj = VideoWriter(outVideoName);
outputVideoObj.FrameRate = 1;%5
open(outputVideoObj);
%%
matPosition = zeros(2*(numFrames(ID) - 4),numFeatures(ID));%matrix containing all feature positions for all frames
numFramesToRead = 5;
%Get the initial state of the feature points
if(ID == 1)
    stateCurrent = getInitialState(videoName{ID}, numFeatures(ID), flowMethods{flowMethodUsed});
elseif(ID == 3)
    stateCurrent = zeros(numFeatures(ID),4);
    ptsFolder = sprintf('%s/../../franck_points/points',videoName{ID});
    for i = 1:3
        fileID = fopen(sprintf('%s/franck_%05d.pts',ptsFolder,i-1));
        %3 useless info
        dummy = fgetl(fileID);
        dummy = fgetl(fileID);
        dummy = fgetl(fileID);

        for j = 1:numFeatures(ID)
            temp = fscanf(fileID,'%f%f',2);
            stateCurrent(j,3:4) = temp' - stateCurrent(j,1:2);
            stateCurrent(j,1:2) = temp';
        end
        fclose(fileID);
    end
else
    stateCurrent = getInitialState(videoName{ID}, numFeatures(ID), flowMethods{flowMethodUsed});
end
%%
kalmanParams = 0;
%Flow estimation and Tracking
for iter1 = 3:numFrames(ID)-2
    fprintf('Processing image: %d\n',iter1);
    for iter2 = 1:numFramesToRead
        if(ID == 3)
            imageSet(:,:,:,iter2) = imread(sprintf('%s/franck_%05d.jpg',videoName{ID},iter1-3+iter2 - 1));
        elseif(ID == 2)
            %Check this
            imageSet(:,:,:,iter2) = imread(sprintf('%s/hotel.seq%d.png',videoName{ID},iter1-3+iter2 - 1));
        else
            imageSet(:,:,:,iter2) = imread(sprintf('%s/%d.png',videoName{ID},iter1-3+iter2));
        end
    end
    
    [stateNext, kalmanParams] = getNextState(stateCurrent, imageSet, flowMethods{flowMethodUsed}, kalmanParams);
    
    %Show images
    if(flowMethodUsed == 3)
        imshow(imageSet(:,:,:,3));
    else
        imshow(imageSet(:,:,:,4));
    end
    hold on;
    for iterPlot = 1:numFeatures(ID)
        plot(stateNext(iterPlot,1), stateNext(iterPlot,2),'Marker','o','MarkerFaceColor','g','MarkerEdgeColor','g');
%         rect = rectangle('Position',[stateNext(iterPlot,1) - 25, stateNext(iterPlot,2) - 25, 50, 50]);
%         rect.LineWidth = 2;
%         rect.EdgeColor = 'r';
    end
    hold off;
    pause(0.001);
    
    figure_1 = getframe;
    [outputImage, map] = frame2im(figure_1);
    %Make a movie
    writeVideo(outputVideoObj,outputImage);
    
    %store all positions in a matrix
    matPosition(2*(iter1-2) - 1:2*(iter1-2),:) = stateNext(:,1:2)';
    stateCurrent = stateNext;
end
close(outputVideoObj);
saveName = sprintf('%s/matPosition_%d.mat',videoName{ID},flowMethodUsed);
save(saveName,'matPosition');
%%
%Perform reconstruction
pts3d_Rigid = perform3DReconstructionRigid(matPosition);
pts3d_NonRigid = perform3DReconstructionNonRigid(matPosition);
%Visualize the recovered 3D data
visualizeData(pts3d_Rigid, pts3d_NonRigid, ID);
figure_1 = getframe;
[outputImage, map] = frame2im(figure_1);
outputImageName = sprintf('%s/Reconstruction_%d.png', videoName{ID}, flowMethodUsed);
saveas(gcf,outputImageName);
% imwrite(outputImage, outputImageName);
end