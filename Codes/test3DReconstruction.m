function test3DReconstruction()
close all;
imageFolder = './Videos/franck_images-0999/images';
ptsFolder = './Videos/franck_points/points';
numImages = 10;
numPts = 68;

matPosition = zeros(numImages*2,numPts);

for i = 1:numImages
    fileID = fopen(sprintf('%s/franck_%05d.pts',ptsFolder,i-1));
    %3 useless info
    dummy = fgetl(fileID);
    dummy = fgetl(fileID);
    dummy = fgetl(fileID);

    for j = 1:numPts
        matPosition(2*i-1:2*i,j) = fscanf(fileID,'%f%f',2);
    end
    fclose(fileID);
end

%Returns only 1 set of 3d points
points3D_1 = perform3DReconstructionRigid(matPosition);

%Returns 3d points for all frames
points3D_2 = perform3DReconstructionNonRigid(matPosition);

%{
figure,
x = points3D_1(1,:)';
y = points3D_1(2,:)';
z = points3D_1(3,:)';
tri = delaunay(x,y);
trisurf(tri,x,y,z)
hold on;
ptsX = [points3D_1(1,32) points3D_1(1,37) points3D_1(1,68)]';
ptsY = [points3D_1(2,32) points3D_1(2,37) points3D_1(2,68)]';
ptsZ = [points3D_1(3,32) points3D_1(3,37) points3D_1(3,68)]';
scatter3(ptsX,ptsY,ptsZ,'filled''MarkerEdgeColor','k','MarkerFaceColor','k');
pts1(:,1) = points3D_1(:,32);
pts1(:,2) = points3D_1(:,37);
pts1(:,3) = points3D_1(:,68);
pts1(:,4) = points3D_1(:,32);
line(pts1(1,:)',pts1(2,:)',pts1(3,:)','color','r','LineWidth',2);
hold off;
%}

figure,
visualize(points3D_1);

figure,
for i = 1:numImages
    visualize(points3D_2(3*i-2:3*i,:));    
%     scatter3(points3D(3*i-2,:)',points3D(3*i-1,:)',points3D(3*i,:)');
%{
    %Option 1
    x = points3D_2(3*i-2,:)';
    y = points3D_2(3*i-1,:)';
    z = points3D_2(3*i,:)';
    tri = delaunay(x,y);
    trisurf(tri,x,y,z)
    hold on;
    ptsX = [points3D_2(3*i-2,31) points3D_2(3*i-2,36)]';
    ptsY = [points3D_2(3*i-1,31) points3D_2(3*i-1,36)]';
    ptsZ = [points3D_2(3*i,31) points3D_2(3*i,36)]';
    scatter3(ptsX,ptsY,ptsZ,'filled''MarkerEdgeColor','k','MarkerFaceColor','k');
    hold off;
%}
    pause(1);
%     keyboard;
end

% %Option 2 - not working
% figure,
% x = points3D(3*i-2,:)';
% y = points3D(3*i-1,:)';
% z = points3D(3*i,:)';
% [xx,yy]=meshgrid(-2:0.1:2,-2:0.1:2);
% zz = griddata(x,y,z,xx,yy);
% surf(xx,yy,zz)

end

function visualize_franck(pts)
scatter3(pts(1,:)',pts(2,:)',pts(3,:)','filled');
hold on;
ptsTemp = pts(:,2:14);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);%Face outline
ptsTemp = pts(:,15:21);
ptsTemp(:,8) = pts(:,15);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','g','LineWidth',2);%person's left eyebrow
ptsTemp = pts(:,1);
ptsTemp(:,2:7) = pts(:,22:27);
ptsTemp(:,8) = pts(:,1);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','y','LineWidth',2);%person's right eyebrow
ptsTemp = pts(:,33:36);
ptsTemp(:,5) = pts(:,33);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','g','LineWidth',2);%person's left eye
ptsTemp = pts(:,28:31);
ptsTemp(:,5) = pts(:,28);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','y','LineWidth',2);%person's right eye
ptsTemp = pts(:,38:46);
ptsTemp(:,10) = pts(:,38);
ptsTemp(:,11) = pts(:,68);
ptsTemp(:,12) = pts(:,46);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);%person's nose
ptsTemp = pts(:,49:60);
ptsTemp(:,13) = pts(:,49);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);%person's mouth
hold off;
end