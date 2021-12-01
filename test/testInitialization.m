%% Test Initialization
% tests initalization

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

% first and third frames
bootstrap_frames = [0, 2];

kitti_path = "data/kitti05/kitti";
malaga_path = "data/malaga-urban-dataset-extract-07";
parking_path = "data/parking";

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load(strcat(kitti_path, '/poses/05.txt'));
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    imsize = [370, 1226];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir(strcat(malaga_path, ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images'));
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load(strcat(parking_path, '/K.txt'));
     
    ground_truth = load(strcat(parking_path, '/poses.txt'));
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end


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


%% Set up camera intrinsics object
focalLenghts = [K(1,1), K(2,2)];
principalPoint = [K(1,3), K(2,3)];
intrinsics = cameraIntrinsics(focalLenghts, principalPoint, imsize);

%% This is here to compare the 3d points to the matched points
[img0Features, img0ValidPoints] = getORBFeatures(img0);
[img1Features, img1ValidPoints] = getORBFeatures(img1);

% uses SSD (Sum of Square Differences) by default
matchIndices = matchFeatures(img0Features, img1Features);

figure; showMatchedFeatures(img0,img1,...
    img0ValidPoints(matchIndices(:,1)),...
    img1ValidPoints(matchIndices(:,2)));
 
%% Initialization
[orbFeatures, landmarks3D, R, T] = bootstrap(img0, img1, intrinsics);


% plots
scatter3(landmarks3D(:,1), landmarks3D(:,2), landmarks3D(:,3));
xlim([-30 30])
ylim([-10 10])
zlim([0 200])
xlabel("x");
ylabel("y");
zlabel("z");
view(0, 90)
title("front view")

figure
scatter3(landmarks3D(:,1), landmarks3D(:,2), landmarks3D(:,3));
xlim([-50 50])
ylim([-50 50])
zlim([0 200])
xlabel("x");
ylabel("y");
zlabel("z");
view(0, 0)
title("top view")
