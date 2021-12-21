fig = createFigure(50);

pause('on')

numPoints = 200;

x = linspace(1,numPoints,numPoints);
y = sin(x);
z = cos(x);

pointX = rand(1,numPoints);
pointY = rand(1,numPoints);

for i = 1:numel(x)
    pause(0.01);

    % update animatedlines
    addpoints(fig.numLandmarksData, x(i), y(i))
    addpoints(fig.fullTrajectoryData, x(i), z(i))

    % update point cloud (animatedlines automatically connects points)
    fig.TopViewX = [fig.TopViewX, pointX(i)];
    fig.TopViewY = [fig.TopViewY, pointY(i)];
    plot(fig.topViewPlot, fig.TopViewX(1:i), fig.TopViewY(1:i), 'o');

    % new camera frame
    img = imread(strcat("data/kitti05/kitti", '/05/image_0/', ...
        sprintf('%06d.png',i)));
    % some modification of the image here...
    imshow(img, 'Parent', fig.currentFramePlot);

    drawnow
end