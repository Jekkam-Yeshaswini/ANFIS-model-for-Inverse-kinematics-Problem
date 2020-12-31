 clear all
 close all

 %% Forward kinematics
 L1 = 10; %non-dimensional unit
 L2 = 7;
 L3 = 5;
 L=[L1,L2,L3];
 %initiliasing identity matrix and symbolic joint angles
 T = eye(3,3);
 theta = sym('q',[1,3]);
 %initialising for loop to run through indices to create final
 %transformation matrice
 for i=1:3
         %creating temporary transform matrix for each set of parameters
         temp = [cos(theta(i)), -sin(theta(i)), cos(theta(i))*L(i); sin(theta(i)), cos(theta(i)), sin(theta(i))*L(i); 0, 0, 1];
         %performing matrix multiplication for transformation matrice
         T = T*temp;
         temp = T(1:2,3);
         JointPos{i} = matlabFunction(temp);
 end
 funcname= matlabFunction(temp)
 %Function JointPos{i} returns the position of the ith joint/frame

 %% Anfis dataset using linspace
 n_q1ANFIS = 20;
 n_q2ANFIS = 20;
 n_q3ANFIS = 20;
 %joints range
 workanglesANFIS = zeros(3, n_q1ANFIS*n_q2ANFIS*n_q3ANFIS);
 EEanglesANFIS = zeros(1, n_q1ANFIS*n_q2ANFIS*n_q3ANFIS);
 i=1;
 for q1 = linspace(0, pi/2, n_q1ANFIS)
    for q2 = linspace(0, pi/2, n_q2ANFIS)
        for q3 = linspace(0, pi/2, n_q3ANFIS)
             workanglesANFIS(1:3,i) = [q1;q2;q3];
             EEanglesANFIS(i) = q1+q2+q3;
             i = i+1;
        end
     end
 end
 %ANFIS training set using end-effector orientation
 EEangletrainingsetThet1 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));EEanglesANFIS; workanglesANFIS(1,:)]';
 EEangletrainingsetThet2 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));EEanglesANFIS; workanglesANFIS(2,:)]';
 EEangletrainingsetThet3 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:)); EEanglesANFIS; workanglesANFIS(3,:)]';
%ANFIS training set without end-effector orientation
 trainingsetThet1 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:)); workanglesANFIS(1,:)]';
 trainingsetThet2 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));workanglesANFIS(2,:)]';
 trainingsetThet3 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));workanglesANFIS(3,:)]';


 %% Overview of the workspace using linspaces
 figure(1)
 title('Reachable workspace of robot : training set for ANFIS')
 workspaceANFIS = JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));
 xx = workspaceANFIS(1,:);
 yy = workspaceANFIS(2,:);
 scatter(xx,yy,5, 'filled');
 set(gca,'DataAspectRatio',[1,1,1])
 colormap(jet);
 %colorbar;

 %% Arm representation
 figure(3);
 Qidle=[0 0 0];
 Joints = JointCart(Qidle,JointPos,[0,0]);
 plot(Joints(1,:), Joints(2,:),'-o','LineWidth',3,'MarkerSize',10,'MarkerFaceColor','#D9FFFF')
 set(gca,'DataAspectRatio',[1,1,1]);
 % title('Idle position - LynxArm') ; xlabel('x (m)') ; ylabel('y (m)');zlabel('z (m)') ;

 %% ANFIS randomized data set
 n_q1ANFIS = 20;
 n_q2ANFIS = 20;
 n_q3ANFIS = 20;
 %joints range
 workanglesANFIS = zeros(3, n_q1ANFIS*n_q2ANFIS*n_q3ANFIS);
 EEanglesANFIS = zeros(1, n_q1ANFIS*n_q2ANFIS*n_q3ANFIS);
 i=1;
 for q1 = linspace(0, pi/2, n_q1ANFIS)
 for q2 = linspace(0, pi/2, n_q2ANFIS)
 for q3 = linspace(0, pi/2, n_q3ANFIS)
 workanglesANFIS(1:3,i) = [rand(1,1)*pi/2;rand(1,1)*pi/2;rand(1,1)*pi/2]; %random angle values (within range)
 EEanglesANFIS(i) = workanglesANFIS(1,i) + workanglesANFIS(2,i) + workanglesANFIS(3,i);
 i = i+1;
 end
 end
 end
 %ANFIS validation set using end-effector orientation
 EEangleRandomsetThet1 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
EEanglesANFIS; workanglesANFIS(1,:)]';
EEangleRandomsetThet2 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
EEanglesANFIS; workanglesANFIS(2,:)]';
EEangleRandomsetThet3 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
EEanglesANFIS; workanglesANFIS(3,:)]';
 %ANFIS training set without end-effector orientation
 RandomsetThet1 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
workanglesANFIS(1,:)]';
RandomsetThet2 = [JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
workanglesANFIS(2,:)]';
RandomsetThet3 =[JointPos{3}(workanglesANFIS(1,:),workanglesANFIS(2,:),workanglesANFIS(3,:));  
workanglesANFIS(3,:)]';

 %% ANFIS training
 genOpt.InputMembershipFunctionType = 'trimf';
 opt = anfisOptions('InitialFIS',[5 5 5],'EpochNumber',15); %Three inputs
 %Gaussian membership functions are selected by default
 opt.DisplayANFISInformation = 0;

 opt.DisplayErrorValues = 1;
 opt.DisplayStepSize = 1;
 opt.DisplayFinalResults = 0;
 opt.ValidationData = EEangleRandomsetThet1(:,:);%Validating data with a whole different data set, with randomly generated points
 [Anfis1,trainError1,stepSize,chkFIS1,chkError1]= anfis(EEangletrainingsetThet1(:,:),opt);
 disp("Anfis1 done")
 opt.ValidationData = EEangleRandomsetThet2(:,:);
 [Anfis2,trainError2,stepSize,chkFIS2,chkError2]= anfis(EEangletrainingsetThet2(:,:),opt);
 disp("Anfis2 done")
 opt.ValidationData = EEangleRandomsetThet3(:,:);
 [Anfis3,trainError3,stepSize,chkFIS3,chkError3]= anfis(EEangletrainingsetThet3(:,:),opt);
 disp("Anfis3 done")


 %% ANFIS errors evaluation and representation
 figure(31)
 tiledlayout(2,2)
 x = [1:15];
 
  
 nexttile
 plot(x,trainError1,'.r',x,chkError1,'.b')
 title('Theta1 (rads)')
 xlabel('Epoch number') 
ylabel('RMSE') 
 legend({'Training','Validation'},'Location','east')
 
 nexttile
 plot(x,trainError2,'.r',x,chkError2,'.b')
 title('Theta2 (rads)')
 xlabel('Epoch number') 
ylabel('RMSE') 
 legend({'Training','Validation'},'Location','east')
 
 nexttile
 plot(x,trainError3,'.r',x,chkError3,'.b')
 title('Theta3 (rads)')
 xlabel('Epoch number') 
 ylabel('RMSE') 
 legend({'Training','Validation'},'Location','east')
 
 %testing the error of the position of the end effector :
 IKTheta1 = evalfis(Anfis1, EEangleRandomsetThet1(:,1:end-1));
 IKTheta2 = evalfis(Anfis2, EEangleRandomsetThet2(:,1:end-1));
 IKTheta3 = evalfis(Anfis3, EEangleRandomsetThet3(:,1:end-1));
 EEPos = JointPos{3}(IKTheta1', IKTheta2', IKTheta3');
 EEPos = EEPos';
 RMSE =sqrt(mean((EEPos(:,1)-EEangleRandomsetThet1(:,1)).^2+(EEPos(:,2)-EEangleRandomsetThet1(:,2)).^2));
maxerror = max(sqrt((EEPos(:,1)-EEangleRandomsetThet1(:,1)).^2+(EEPos(:,2)-EEangleRandomsetThet1(:,2)).^2));
disp("Absolute RMSE")
 disp(RMSE)
 disp("Max absolute End effector error")
 disp(maxerror)

 figure(30) %plotting error in the workspace
 %color vector
 %disp(EEPos)
 %disp(EEangleRandomsetThet1)
 %disp(ErrorVector)
 ErrorVector = sqrt((EEPos(:,1)-EEangleRandomsetThet1(:,1)).^2+(EEPos(:,2)-EEangleRandomsetThet1(:,2)).^2);
 tri = delaunay(EEangleRandomsetThet1(:,1),EEangleRandomsetThet1(:,2));

 h = trisurf(tri, EEangleRandomsetThet1(:,1), EEangleRandomsetThet1(:,2), ErrorVector);
 axis vis3d
 title("Value of the end-effector position error throughout the workspace")
 colormap(jet);
 
 pause

 
 function [Cart] = JointCart(Q,JointPos,Origin) %Performs an FK for all the joints, in  order to plot the whole arm
 Cart1 = JointPos{1}(Q(1));
 Cart2 = JointPos{2}(Q(1),Q(2));
 Cart3 = JointPos{3}(Q(1),Q(2),Q(3));
 Cart = [Origin', Cart1+Origin , Cart2+Origin , Cart3+Origin];
end