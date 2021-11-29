%% testORBFeatures
% tests FAST feature detection and ORB feature extraction
% uses code from bootstrapping

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

% first and third frames
bootstrap_frames = [0, 2];

kitti_path = "data/kitti05/kitti";
malaga_path = "data/malaga-urban-dataset-extract-07";
parking_path = "data/parking";

%% Get frames for test
% need to set bootstrap_frames
if ds == 0
    img0 = imread(strcat(kitti_path, '/05/image_0/', ...
        sprintf('%06d.png',bootstrap_frames(1))));
    img1 = imread(strcat(kitti_path, '/05/image_0/', ...
        sprintf('%06d.png',bootstrap_frames(2))));
elseif ds == 1
    img0 = rgb2gray(imread(strcat(malaga_path, ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/', ...
        left_images(bootstrap_frames(1)).name)));
    img1 = rgb2gray(imread(strcat(malaga_path, ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/', ...
        left_images(bootstrap_frames(2)).name)));
elseif ds == 2
    img0 = rgb2gray(imread(strcat(parking_path, ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1)))));
    img1 = rgb2gray(imread(strcat(parking_path, ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2)))));
else
    assert(false);
end

%% Extract ORB features, visualize

[img0Features, img0ValidPoints] = getORBFeatures(img0);
[img1Features, img1ValidPoints] = getORBFeatures(img1);

% uses SSD (Sum of Square Differences) by default
matchIndices = matchFeatures(img0Features, img1Features);

figure; showMatchedFeatures(img0,img1,...
    img0ValidPoints(matchIndices(:,1)),...
    img1ValidPoints(matchIndices(:,2)));