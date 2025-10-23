% Test for helper script for lab 2 of MVG section 4
clear all;
close all;
clc;


%image1filename = 'DataSet01/00.png';
%image2filename = 'DataSet01/02.png';
%load('DataSet01/Features.mat');
%CL1uv = Features(1).xy;
%CL2uv = Features(3).xy;

%I1 = imread(image1filename);
%I2 = imread(image2filename);

%Model = 'Affine';
%H12 = computeHomography(CL1uv,CL2uv, Model);

% This is an alternative way of showing the associations, using a toolbox function
%figure;
%showMatchedFeatures(I1, I2, CL1uv, CL2uv, 'Montage');

% Usage of computeReprojectionError
%visualizeReprojectionError(I1, I2, CL1uv, CL2uv, Model);


% This is a function that warps image I2 into the frame of image I1 and
% shows the result with red and green colors
%figure; 
%showwarpedimages(I1, I2, H12);




% After testing with the synthetic images you can test with the underwater
% images

image1filename = 'imgl01311.jpg';
image2filename = 'imgl01396.jpg';
distRatio = 0.8; [CL1uv,CL2uv] = matchsiftmodif(image1filename, image2filename, distRatio,false);

I1 = imread(image1filename);
I2 = imread(image2filename);

Model = 'Affine';
[H12, INLIERS1uv, INLIERS2uv] = computeHomographyRANSAC(CL1uv, CL2uv, Model);

% Visualize reprojection error
visualizeReprojectionErrorWithRANSAC(I1, I2, CL1uv, CL2uv, Model);

% This is an alternative way of showing the associations, using a toolbox function
figure;
showMatchedFeatures(I1, I2, CL1uv, CL2uv, 'Montage');
figure; 
showwarpedimages(I1, I2, H12);


return;