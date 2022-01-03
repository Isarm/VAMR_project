numFrames = 20;

[fig, topViewLandmarksX, topViewLandmarksY, topViewCarX, topViewCarY] = createFigure(50, numFrames);
topViewLastCell = 1;

pause('on')

numPoints = 200;

x = linspace(1,numPoints,numPoints);
y = sin(x);
z = cos(x);

pointX = rand(1,numPoints);
pointY = rand(1,numPoints);
pointX2 = rand(1,numPoints);
pointY2 = rand(1,numPoints);

%for i = 1:numel(x)
for i = 1:100
    pause(0.01);

    % new camera frame
    img = imread(strcat("data/kitti05/kitti", '/05/image_0/', ...
        sprintf('%06d.png',i)));
    % some modification of the image here...

    % annoying work we need to do because pass-by-value, this will be in the final script
    % since actual data is a single
    topViewLandmarksX{topViewLastCell} = single(pointX(i));
    topViewLandmarksY{topViewLastCell} = single(pointY(i));
    topViewCarX{topViewLastCell} = pointX2(i);
    topViewCarY{topViewLastCell} = pointY2(i);
    topViewLastCell = mod(topViewLastCell+1, numFrames+1);
    if topViewLastCell == 0
        topViewLastCell = 1;
    end

    % just for test
    landMarkPositionsX = cell2mat(topViewLandmarksX)' * 1226;
    landMarkPositionsY = cell2mat(topViewLandmarksY)' * 370;

    % lots of transposes and concatenation because x and y are separate structures in this little test script; when extracting keypoints using matlab's built-in functions, these will already be in the right kx2 format
    updateFigure(fig, img, [landMarkPositionsX,landMarkPositionsY], y(i), [x(i), z(i)], topViewLandmarksX, topViewLandmarksY, topViewCarX, topViewCarY);
end