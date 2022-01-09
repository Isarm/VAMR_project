%% VAMR Mini-Project
% ...

%% 
clc
clear
close all

%% [INPUTS] 
% Dataset Selection
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

% TODO: For each datast, different bootstrap frames
bootstrapFrame1 = 0;
bootstrapFrame2 = 2;

numPointsNumLandmarks = 50;
numFrames = 20;

%% Setup
parameters = getParameters();
[img1, img2, intrinsics] = getInitialFrames(ds, bootstrapFrame1, bootstrapFrame2);
[fig, topViewLandmarksX, topViewLandmarksZ, topViewCarX, topViewCarZ] = createFigure(numPointsNumLandmarks, numFrames);

%% Bootstrap
[keyPoints, landmarks3D, R, T] = bootstrap(img1, img2, intrinsics);

%% Continuous operation
% Initialize State Variable
S = struct;
S.P = keyPoints;
S.X = landmarks3D;
S.C = [];
S.F = [];
S.T = [];

while true
    % Makes sure that plots refresh.    
    pause(0.01);
end

% range = (bootstrap_frames(2)+1):last_frame;
% for i = range
%     fprintf('\n\nProcessing frame %d\n=====================\n', i);
%     if ds == 0
%         image = imread(strcat(kitti_path, '/05/image_0/', sprintf('%06d.png',i)));
%     elseif ds == 1
%         image = rgb2gray(imread(strcat(malaga_path, ...
%             '/malaga-urban-dataset-extract-07_rectified_800x600_Images/', ...
%             left_images(i).name)));
%     elseif ds == 2
%         image = im2uint8(rgb2gray(imread(strcat(parking_path, ...
%             sprintf('/images/img_%05d.png',i)))));
%     else
%         assert(false);
%     end
%     % Makes sure that plots refresh.    
%     pause(0.01);
%     
%     prev_img = image;
% end
