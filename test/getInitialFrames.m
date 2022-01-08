function [img0, img1, intrinsics] = getInitialFrames(dataset, frame1, frame2)
    % Returns pair of initial frames from chosen dataset, as well as the camera instrinsics for the given dataset
    % Inputs: 
    %   dataset = integer representing dataset (see below)
    %   frame1  = first frame to use (begins at 0)
    %   frame2  = second frame to use (begins at frame1+1 or further)

    %% Setup

    ds = dataset; % 0: KITTI, 1: Malaga, 2: parking

    % first and third frames
    bootstrap_frames = [frame1, frame2];

    kitti_path = "data/kitti05/kitti";
    malaga_path = "data/malaga-urban-dataset-extract-07";
    parking_path = "data/parking";
    custom_path = "data/custom";
    custom_high_fov_path = "data/custom_high_fov";

    if ds == 0
        % need to set kitti_path to folder containing "05" and "poses"
        assert(exist('kitti_path', 'var') ~= 0);
        ground_truth = load(strcat(kitti_path, '/poses/05.txt'));
        ground_truth = ground_truth(:, [end-8 end]);
        last_frame = 4540;
        K = [7.188560000000e+02 0 6.071928000000e+02
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];
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
    elseif ds == 3
        % Path containing images, depths and all...
        assert(exist('custom_path', 'var') ~= 0);
        last_frame = 598;
        cameraParams = load(strcat(custom_path, '/calibration/cameraParams.mat'));
        K = cameraParams.cameraParams.IntrinsicMatrix';
    elseif ds == 4
        % Path containing images, depths and all...
        assert(exist('custom_high_fov_path', 'var') ~= 0);
        last_frame = 404;
        cameraParams = load(strcat(custom_high_fov_path, '/calibration/cameraParams.mat'));
        K = cameraParams.cameraParams.IntrinsicMatrix';
    else
        assert(false);
    end


    %% Bootstrap
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
    elseif ds == 3
        img0 = rgb2gray(imread(strcat(custom_path, ...
            sprintf('/images/img_%05d.jpg',bootstrap_frames(1)))));
        img1 = rgb2gray(imread(strcat(custom_path, ...
            sprintf('/images/img_%05d.jpg',bootstrap_frames(2)))));
    elseif ds == 4
        img0 = rgb2gray(imread(strcat(custom_high_fov_path, ...
            sprintf('/images/img_%05d.jpg',bootstrap_frames(1)))));
        img1 = rgb2gray(imread(strcat(custom_high_fov_path, ...
            sprintf('/images/img_%05d.jpg',bootstrap_frames(2)))));
    else
        assert(false);
    end

    %% Set up camera intrinsics object
    focalLenghts = [K(1,1), K(2,2)];
    principalPoint = [K(1,3), K(2,3)];
    intrinsics = cameraIntrinsics(focalLenghts, principalPoint, size(img0));
end
