clear all; close all; clc;

% Initialize VideoReader
videoFile = 'addyourcrowd.mp4';
vidObj = VideoReader("addyourcrowd.mp4");

% Define your frame dimensions
frameWidth = 50;  % Define the width of your frame
frameHeight = 50;  % Define the height of your frame


% Initialize SP model to store coordinates
SP_model = struct('Time', {}, 'Coordinates', {});

% Parameters
frameRate = vidObj.FrameRate;
timeInterval = 1;  % Interval for saving coordinates (in seconds)
timeCounter = 0;

% Initialize variables for tracking
prevCoordinates = [];

% Blue color range (adjust these values based on your video)
lowerBlue = [0, 0, 100]; % Lower bound of blue color in RGB
upperBlue = [50, 50, 255]; % Upper bound of blue color in RGB

% K-Means clustering parameters
clusterDistanceThreshold = 20;  % Adjust this threshold based on object separation



% Loop through video frames
while hasFrame(vidObj)
    frame = readFrame(vidObj);

    % Convert the frame to grayscale
    grayFrame = rgb2gray(frame);

    % Create a binary mask for blue objects
    blueMask = (frame(:,:,1) >= lowerBlue(1) & frame(:,:,1) <= upperBlue(1)) & ...
               (frame(:,:,2) >= lowerBlue(2) & frame(:,:,2) <= upperBlue(2)) & ...
               (frame(:,:,3) >= lowerBlue(3) & frame(:,:,3) <= upperBlue(3));

    % Find blue pixels
    [blueY, blueX] = find(blueMask);

    % Combine x and y coordinates into a single array
    blueCoordinates = [blueX, blueY];

    % Perform K-Means clustering to identify pedestrians
    if ~isempty(blueCoordinates)
        % Apply K-Means clustering with a specified distance threshold
        [clusterIdx, clusterCenters] = clusterBluePixels(blueCoordinates, clusterDistanceThreshold);

        % Scale coordinates based on frame dimensions
        if ~isempty(clusterCenters)
            clusterCenters(:, 1) = clusterCenters(:, 1) / size(frame, 2) * frameWidth;
            clusterCenters(:, 2) = clusterCenters(:, 2) / size(frame, 1) * frameHeight;
        end

        % Store cluster centers as pedestrian coordinates
        coordinates = clusterCenters;
    else
        coordinates = [];
    end
    
    % Extract and store coordinates of detected and tracked pedestrians
    if ~isempty(coordinates)
        timeCounter = timeCounter + 1 / frameRate;
        
        % Save coordinates every 1 second
        if timeCounter >= timeInterval
            SP_model(end + 1).Time = vidObj.CurrentTime;
            SP_model(end).Coordinates = coordinates;
            timeCounter = 0;  % Reset the counter
        end
    end
    
    % Display tracking results (optional)
    frame(blueMask) = 255; % Highlight blue pixels
    imshow(frame);
    drawnow;
    
    % Update previous frame variables
    prevCoordinates = coordinates;
end

% Cleanup (no need for release(vidObj) in this case)
clear vidObj;

% Save SP model as a .mat file
save('SP_model.mat', 'SP_model');

% Custom function for K-Means clustering with a distance threshold
function [clusterIdx, clusterCenters] = clusterBluePixels(coordinates, threshold)
    % Initialize clustering
    numPoints = size(coordinates, 1);
    clusterIdx = zeros(numPoints, 1);
    clusterCenters = [];

    % Iterate through points and perform clustering
    for i = 1:numPoints
        point = coordinates(i, :);

        % Check if the point is close to any existing cluster center
        if ~isempty(clusterCenters)
            dists = sqrt(sum((clusterCenters - point).^2, 2));
            minDist = min(dists);

            if minDist <= threshold
                % Assign the point to the nearest existing cluster
                [~, idx] = min(dists);
                clusterIdx(i) = idx;
                continue;
            end
        end

        % Create a new cluster
        clusterIdx(i) = size(clusterCenters, 1) + 1;
        clusterCenters = [clusterCenters; point];
    end
end