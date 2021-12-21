function fig = createFigure(numPointsNumLandmarks)
    fig = struct;
    fig.f = figure;

    fig.currentFramePlot = subplot(2,2,1);
    fig.currentFrameData = [];

    fig.topViewPlot = subplot(2,2,2);
    % should only be tracking last 20 frames, so need some clever way of tracking and updating these arrays (efficiently)
    fig.TopViewX = [];
    fig.TopViewY = [];
    axis equal;

    fig.numLandmarksPlot = subplot(2,2,3);
    fig.numLandmarksData = animatedline('MaximumNumPoints',numPointsNumLandmarks);
    axis equal;

    fig.fullTrajectoryPlot = subplot(2,2,4);
    fig.fullTrajectoryData = animatedline;
    axis equal;
end