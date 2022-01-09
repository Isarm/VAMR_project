ds = 3;

custom_path = "data/custom";
custom_high_fov_path = "data/custom_high_fov";

if ds == 3
    cameraParams = load(strcat(custom_path, '/calibration/cameraParams.mat'));
    cameraParams = cameraParams.cameraParams;
    for i=1:260
        img = imread(strcat(custom_path, ...
            sprintf('/images_distorted/img_%05d.jpg',i)));
        img_und = undistortImage(img, cameraParams, 'OutputView', 'full');
        imshow(img);
        imwrite(img_und, strcat(custom_path, ...
            sprintf('/images/img_%05d.jpg', i)));
    end
elseif ds == 4
    cameraParams = load(strcat(custom_high_fov_path, '/calibration/cameraParams.mat'));
    cameraParams = cameraParams.cameraParams;
    for i=1:404
        img = imread(strcat(custom_high_fov_path, ...
            sprintf('/images_distorted/img_%05d.jpg',i)));
        img_und = undistortImage(img, cameraParams, 'OutputView', 'full');
        imshow(img_und);
        imwrite(img_und, strcat(custom_high_fov_path, ...
            sprintf('/images/img_%05d.jpg', i)));
    end
end
