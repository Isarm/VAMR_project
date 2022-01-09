function [img] = getFrame(ds, frame)
% Gets frame number $frame from $ds

    kitti_path = "data/kitti05/kitti";
    malaga_path = "data/malaga-urban-dataset-extract-07";
    parking_path = "data/parking";
    custom_path = "data/custom";
    custom_high_fov_path = "data/custom_high_fov";

    if ds == 0
        img = imread(strcat(kitti_path, '/05/image_0/', ...
            sprintf('%06d.png',frame)));
    elseif ds == 1
        images = dir(strcat(malaga_path, ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images'));
        left_images = images(3:2:end);
        img = rgb2gray(imread(strcat(malaga_path, ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/', ...
            left_images(frame).name)));
    elseif ds == 2
        img = rgb2gray(imread(strcat(parking_path, ...
            sprintf('/images/img_%05d.png',frame))));
    elseif ds == 3
        img = rgb2gray(imread(strcat(custom_path, ...
            sprintf('/images/img_%05d.jpg',frame))));
    elseif ds == 4
        img = rgb2gray(imread(strcat(custom_high_fov_path, ...
            sprintf('/images/img_%05d.jpg',frame))));
    else
        assert(false);
    end
end

