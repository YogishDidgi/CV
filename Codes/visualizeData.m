function visualizeData(pts3d_Rigid, pts3d_NonRigid, id)
figure,
if(id == 2)
    subplot(1,3,1),visualize_hotel(pts3d_Rigid);title('Rigid 3D Reconstruction');view(-120,90);
    subplot(1,3,2),visualize_hotel(pts3d_NonRigid(1:3,:));title('Non-Rigid 3D Reconstruction');view(-120,90);
    subplot(1,3,3),visualize_hotel(pts3d_NonRigid(end-2:end,:));title('Non-Rigid 3D Reconstruction');view(-120,90);
elseif(id == 3)
    subplot(1,3,1),visualize_franck(pts3d_Rigid);title('Rigid 3D Reconstruction');view(-91,-90);
    subplot(1,3,2),visualize_franck(pts3d_NonRigid(1:3,:));title('Non-Rigid 3D Reconstruction');view(91,-90);
    subplot(1,3,3),visualize_franck(pts3d_NonRigid(end-2:end,:));title('Non-Rigid 3D Reconstruction');view(91,-90);
elseif(id == 4)
    subplot(1,3,1),visualize_boxofjoe(pts3d_Rigid);title('Rigid 3D Reconstruction');view(-80,-90);
    subplot(1,3,2),visualize_boxofjoe(pts3d_NonRigid(1:3,:));title('Non-Rigid 3D Reconstruction');view(100,-90);
    subplot(1,3,3),visualize_boxofjoe(pts3d_NonRigid(end-2:end,:));title('Non-Rigid 3D Reconstruction');view(100,-90);
elseif(id == 5)
    subplot(1,3,1),visualize_paper(pts3d_Rigid);title('Rigid 3D Reconstruction');view(130,-90);
    subplot(1,3,2),visualize_paper(pts3d_NonRigid(1:3,:));title('Non-Rigid 3D Reconstruction');view(70,90);
    subplot(1,3,3),visualize_paper(pts3d_NonRigid(end-2:end,:));title('Non-Rigid 3D Reconstruction');view(70,90);
elseif(id == 6)
    subplot(1,3,1),visualize_gyan(pts3d_Rigid);title('Rigid 3D Reconstruction');view(140,-90);
    subplot(1,3,2),visualize_gyan(pts3d_NonRigid(1:3,:));title('Non-Rigid 3D Reconstruction');view(140,-90);
    subplot(1,3,3),visualize_gyan(pts3d_NonRigid(end-2:end,:));title('Non-Rigid 3D Reconstruction');view(140,-90);
end

end

function visualize_hotel(pts)
scatter3(pts(1,:)',pts(2,:)',pts(3,:)','filled');
hold on;
ptsTemp = pts(:,1:2);
ptsTemp(:,3) = pts(:,4);
ptsTemp(:,4) = pts(:,3);
ptsTemp(:,5) = pts(:,1);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,5:6);
ptsTemp(:,3) = pts(:,8);
ptsTemp(:,4) = pts(:,7);
ptsTemp(:,5) = pts(:,5);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,9:10);
ptsTemp(:,3) = pts(:,12);
ptsTemp(:,4) = pts(:,11);
ptsTemp(:,5) = pts(:,9);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,1);
ptsTemp(:,2) = pts(:,5);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,2);
ptsTemp(:,2) = pts(:,6);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
hold off;
end

function visualize_boxofjoe(pts)
scatter3(pts(1,:)',pts(2,:)',pts(3,:)','filled');
hold on;
ptsTemp = pts(:,1:4);
ptsTemp(:,5) = pts(:,1);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,1);
ptsTemp(:,2:4) = pts(:,9:11);
ptsTemp(:,5) = pts(:,3);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,10);
ptsTemp(:,2) = pts(:,2);
ptsTemp(:,3:4) = pts(:,12:13);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,7);
ptsTemp(:,2:3) = pts(:,5:6);
ptsTemp(:,4) = pts(:,8);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,1);
ptsTemp(:,2) = pts(:,7);
ptsTemp(:,3) = pts(:,4);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,2);
ptsTemp(:,2) = pts(:,8);
ptsTemp(:,3) = pts(:,3);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
hold off;
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

function visualize_gyan(pts)
scatter3(pts(1,:)',pts(2,:)',pts(3,:)','filled');
hold on;
ptsTemp = pts(:,1:3);
ptsTemp(:,4) = pts(:,6);
ptsTemp(:,5) = pts(:,5);
ptsTemp(:,6) = pts(:,2);
ptsTemp(:,7) = pts(:,4);
ptsTemp(:,8) = pts(:,6);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','y','LineWidth',2);
ptsTemp = pts(:,7:9);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','g','LineWidth',2);
ptsTemp = pts(:,10:11);
ptsTemp(:,3) = pts(:,13);
ptsTemp(:,4) = pts(:,12);
ptsTemp(:,5) = pts(:,10);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','r','LineWidth',2);
ptsTemp = pts(:,2);
ptsTemp(:,2) = pts(:,10);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','b','LineWidth',2);
ptsTemp = pts(:,9);
ptsTemp(:,2) = pts(:,13);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','b','LineWidth',2);
hold off;
end

function visualize_paper(pts)
scatter3(pts(1,:)',pts(2,:)',pts(3,:)','filled');
hold on;
ptsTemp = pts(:,1:2);
ptsTemp(:,3) = pts(:,4);
ptsTemp(:,4) = pts(:,3);
ptsTemp(:,5) = pts(:,1);
ptsTemp(:,6:8) = pts(:,5:7);
ptsTemp(:,9) = pts(:,4);
ptsTemp(:,10) = pts(:,3);
ptsTemp(:,11) = pts(:,6);
line(ptsTemp(1,:)',ptsTemp(2,:)',ptsTemp(3,:)','color','g','LineWidth',2);
hold off;
end