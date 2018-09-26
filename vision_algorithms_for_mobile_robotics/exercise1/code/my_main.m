%close all;
clear all;

% Load camera poses
% Each row i of matrix 'poses' contains the transformations that transforms
% points expressed in the world frame to points expressed in the camera
% frame.
pose_vectors = load('../data/poses.txt');

% Define 3D corner positions
% [Nx3] matrix containing the corners of the checkerboard as 3D points
% (X, Y, Z), expressed in the world coordinate system.
square_size = 0.04
num_corners_x = 9; num_corners_y = 6;
num_corners = num_corners_x * num_corners_y;

[X, Y] = meshgrid(0:num_corners_x-1,0:num_corners_y-1);
p_W_corners = square_size * [X(:) Y(:)];
p_W_corners = [p_W_corners zeros(num_corners,1)]';

% Load camera intrinsics
K = load('../data/K.txt'); % calibration matrix [3x3]
D = load('../data/D.txt'); % distortion coefficients [2x1]

% Load one image with a given index
img_index = 1;
img = rgb2gray(imread(['../data/images/', sprintf('img_%04d.jpg',img_index)]));

% Project the conrners on the image
% Compute the 4x4 homogeneous transformation matrix that maps points from
% the world to the camera coordinate frame
T_C_W = myPoseVectorToTransformationMatrix(pose_vectors(img_index,:));
p_C_corners = T_C_W * [p_W_corners; ones(1,num_corners)];
projected_pts = myProjectPoints(p_C_corners,K,D);
figure();
imshow(img);hold on;
plot(projected_pts(1,:),projected_pts(2,:),'r.');
drawCube(T_C_W,K,D);
% Undistort image with bilinear interpolation
tic;
img_undistorted = myUndistortImage(img, K, D,1);
disp(['Undistortin with bilinear interpolation completed in ' num2str(toc)]);

figure();
%subplot(1,2,1);
imshow(img_undistorted);