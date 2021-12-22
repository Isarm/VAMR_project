function [fig, topViewLandmarksX, topViewLandmarksY, topViewCarX, topViewCarY] = createFigure(numPointsNumLandmarks, numFrames)
    %CREATEFIGURE
    % Creates and returns a figure which can be updated with each frame processed

    % Inputs
    %
    %   numPointsNumLandmarks   = number of points to show in the Number of Landmarks plot before sliding the window
    %   numFrames               = number of frame data to show in the top-view plot
    
    % Outputs
    %
    %   fig                 = figure itself
    %   topViewLandmarksX   = cell array to update landmarks to show in last numFrames frames
    %   topViewLandmarksY   = cell array to update landmarks to show in last numFrames frames
    %   topViewCarX        = cell array to update car positions to show in last numFrames frames
    %   topViewCarY        = cell array to update car positions to show in last numFrames frames

    fig = struct;
    fig.f = figure;
    fig.numFrames = numFrames;

    % Top left corner: plot current frame and keypoints or whatever else we'd like
    fig.currentFramePlot = subplot(2,2,1);
    title('Current Image')
    hold(fig.currentFramePlot, 'on')

    % Top right corner: plot 3D landmarks and car position as seen from above
    fig.topViewPlot = subplot(2,2,2);
    % TODO: get this title to show?
    title(sprintf('Trajectory of last %d frames and landmarks', numFrames));
    axis equal;
    % each cell contains the values for a given frame
    % since matlab is pass by value only, we need to update these outside this function
    topViewLandmarksX = cell(1,numFrames);
    topViewLandmarksY = cell(1,numFrames);
    topViewCarX = cell(1,numFrames);
    topViewCarY = cell(1,numFrames);
    
    % Bottom left corner: number of keypoints tracked over the last numFrames frames
    fig.numLandmarksPlot = subplot(2,2,3);
    fig.numLandmarksData = animatedline('MaximumNumPoints',numPointsNumLandmarks);
    xlim([-numFrames, 0])
    title(sprintf('Number of landmarks tracked over last %d frames', numFrames));
    addpoints(fig.numLandmarksData, 0, 0)
    axis equal;

    % Bottom right corner: position of vehicle over time?
    fig.fullTrajectoryPlot = subplot(2,2,4);
    fig.fullTrajectoryData = animatedline;
    title('Full Trajectory')
end