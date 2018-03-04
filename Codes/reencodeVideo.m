function reencodeVideo(inVideo, outVideo)
inputVideoObj = VideoReader(inVideo);
% open(inputVideoObj);
outputVideoObj = VideoWriter(outVideo);
outputVideoObj.FrameRate = 1;
open(outputVideoObj);

while hasFrame(inputVideoObj)
    image = readFrame(inputVideoObj);
    writeVideo(outputVideoObj,image);
end

% close(inputVideoObj);
close(outputVideoObj);
fprintf('Done\n');
end