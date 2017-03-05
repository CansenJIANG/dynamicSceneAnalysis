clear;
close all;


% Parameters
localRelativeThreshold = 0.4;
maximumHoleSizeInPercent = 0.01 / 100;
refinementKernelHalfSize = 2;
nrCornersX = 6;
nrCornersY = 9;

% Load example image and perform corner detection
image = double(rgb2gray(imread('example.png'))) / 255;

[rc rs c] = rochade(image, nrCornersX, nrCornersY, refinementKernelHalfSize);

imshow(image);
hold on;
plot(rc(:,1), rc(:,2), '-r');
plot(rc(:,1), rc(:,2), '+g');
