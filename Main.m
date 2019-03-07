% Title : Trying to use sum of squared errors for Triangulation
% Author: Shreyash Annapureddy
% Date  : 14/03/2016
%
%
%
%
%
clear all
close all
clc
%% var parameters to be used

% Intrinsic parameters in terms of pixels
intrin_param = [727.65579,0,330.50635;0,727.95799,167.87330;0,0,1];
fx  = intrin_param(1,1);
fy  = intrin_param(2,2);
o_x = intrin_param(1,3);
o_y = intrin_param(2,3);

Focal = 2.3; % Focal length in mm of webCam

mm_x = 1/(fx/Focal); % Conversion factor mm/pixels
mm_y = 1/(fy/Focal); % Conversion factor mm/pixels
pix2mm = [mm_x;mm_y;1];

camParam = struct('fx',fx,...
                  'fy',fy,...
                  'ox',o_x,...
                  'oy',o_y,...
                  'k1',0.03457,...
                  'k2',-0.34556,...
                  'k3',0.000,...
                  'p1',0.0058,...
                  'p2',0.00316,...
                  'F',Focal);
              
%% Load Images

img_dir = 'C:\Abyss Solutions\My Own Odometry\Matlab Prototype\Image Data\WebCam Data\MovementTest_4';
first_frame = 1;
next_frame = 2;

img1 = imread([img_dir '\SimpleMove' num2str(first_frame) '.JPG']);
img2 = imread([img_dir '\SimpleMove' num2str(next_frame) '.JPG']);

img1_gray = rgb2gray(img1);
img2_gray = rgb2gray(img2);

img1_gray = undistort(im2double(img1_gray),camParam); % Undistorting img1
img2_gray = undistort(im2double(img2_gray),camParam); % Undistorting img2

[yDim,xDim] = size(img1_gray);


%% Create a simulates frame of cam sensor
frame_init = [0,xDim*mm_x,xDim*mm_x,0,0;...
     0,0,yDim*mm_y,yDim*mm_y,0;...
     camParam.F,camParam.F,camParam.F,camParam.F,camParam.F;...
     1,1,1,1,1];

%% SIFT Feature extraction

[f1,d1] = SIFT(img1);
[f2,d2] = SIFT(img2);

[matches,scores] = vl_ubcmatch(d1,d2);

c1 = f1(1,matches(1,:));
r1 = f1(2,matches(1,:));

c2 = f2(1,matches(2,:));
r2 = f2(2,matches(2,:));

x1 = [c1; r1; ones(1,length(c1))];
x2 = [c2; r2; ones(1,length(c2))];

%% Plot Features detected in SIFT
imshow(img1_gray);
h1 = vl_plotframe(f1(:,:));
h2 = vl_plotframe(f1(:,:));
set(h1,'color','k','linewidth',3) ;
set(h2,'color','y','linewidth',2) ;

figure 
imshow(img2_gray);
h1 = vl_plotframe(f2(:,:));
h2 = vl_plotframe(f2(:,:));
set(h1,'color','k','linewidth',3) ;
set(h2,'color','y','linewidth',2) ;

%% Determining the Fundemental Matrix
threshold   = 0.001;
[F,inliers] = ransacfitfundmatrix(x1, x2, threshold);

%% Determining the Essential Matrix
E = intrin_param'*F*intrin_param;
[transform_Matrices] = EssentialMatrixToCameraMatrix(E);

%% Force Transformations matrices to particular form
transform_Matrices_forced = ConstrainTransformations(transform_Matrices);

%% Plot the SIFT matches based on Inliers found by RANSAC

show(double(img1_gray)+ double(img2_gray),4), set(4,'name','Inlying matches'), hold on    
plot(c1(inliers(10)),r1(inliers(10)),'r+');
plot(c2(inliers(10)),r2(inliers(10)),'g+');    

for n = inliers
    line([c1(n) c2(n)], [r1(n) r2(n)],'color',[0 0 1])
end


%% Plot the frame after transformation

feature_Set(:,:,1) = [c1(inliers(10));r1(inliers(10));camParam.F];
feature_Set(:,:,2) = [c2(inliers(10));r2(inliers(10));camParam.F];

for  i = 1:4
    figure;
    [transFeatures,transOptCnt,ProjPoints] = Projection_Points(transform_Matrices_forced(:,:,i),...
                                                           camParam,feature_Set,...
                                                           pix2mm);
    PlotFrames(transform_Matrices_forced(:,:,i),frame_init);
    hold on
    plot3(transOptCnt(1,1),transOptCnt(2,1),transOptCnt(3,1),'rx','MarkerSize',8);
    hold on
    plot3(transFeatures(1,1),transFeatures(2,1),transFeatures(3,1),'ro','MarkerSize',8)
    hold on
    plot3(ProjPoints(1,:,1),ProjPoints(2,:,1),ProjPoints(3,:,1),'r')
    hold on

    plot3(transOptCnt(1,2),transOptCnt(2,2),transOptCnt(3,2),'bx','MarkerSize',8);
    hold on
    plot3(transFeatures(1,2),transFeatures(2,2),transFeatures(3,2),'bo','MarkerSize',8);
    hold on
    plot3(ProjPoints(1,:,2),ProjPoints(2,:,2),ProjPoints(3,:,2),'b')
    hold off;
    
end

%% Triangulation Old Method
[t1,t2,point_3D] = Triangulation(transform_Matrices_forced,...
                                     feature_Set,...
                                     intrin_param,...
                                     pix2mm)
                                 

