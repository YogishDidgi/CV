%Extract images from a video and write to a folder
%Inputs
    %arg1 - video name
    %arg2 - folder location where images will be saved
%Ouputs
    %
function extractFrames(arg1, arg2)
videoReaderObj = VideoReader(arg1);
lastFrame = read(videoReaderObj, inf);
numFrames = videoReaderObj.NumberOfFrames;
videoReaderObj = VideoReader(arg1);
for i = 1:numFrames
    imageFrame = readFrame(videoReaderObj);
    imageName = sprintf('%s/%d.png',arg2,uint32(i));
    imwrite(imageFrame, imageName);
end

end