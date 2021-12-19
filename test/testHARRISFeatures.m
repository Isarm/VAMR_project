%% testHARRISFeatures

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
elseif ds == 1
    img0 = rgb2gray(imread(strcat(malaga_path, ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/', ...
        left_images(bootstrap_frames(1)).name)));
elseif ds == 2
    img0 = rgb2gray(imread(strcat(parking_path, ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1)))));
else
    assert(false);
end

N = getHarrisFeatures(img1);

figure;
imagesc(img1)
hold on
plot(N)
