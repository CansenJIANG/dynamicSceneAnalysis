%% KLT tracker Matlab
% Create System objects for reading and displaying video and for drawing a bounding box of the object.

%% Initialization
videoFileReader = vision.VideoFileReader('visionface.avi');
videoPlayer = vision.VideoPlayer('Position', [100, 100, 680, 520]);


% Read the first video frame, which contains the object, define the region.
objectFrame = step(videoFileReader);
objectRegion = [264, 122, 93, 93];

% As an alternative, you can use the following commands to select the object region using a mouse. The object must occupy the majority of the region.

figure; imshow(objectFrame); objectRegion=round(getPosition(imrect))

% Show initial frame with a red bounding box.

objectImage = insertShape(objectFrame, 'Rectangle', objectRegion,'Color', 'red');
figure; imshow(objectImage); title('Yellow box shows object region');


%% Feature detection
% Detect interest points in the object region.
points = detectMinEigenFeatures(rgb2gray(objectFrame), 'ROI', objectRegion);

% Display the detected points.
pointImage = insertMarker(objectFrame, points.Location, '+', 'Color', 'white');
figure, imshow(pointImage), title('Detected interest points');

%% Feature tracking
% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError', 1);


% Initialize the tracker.
initialize(tracker, points.Location, objectFrame);

% Read, track, display points, and results in each video frame.
while ~isDone(videoFileReader)
      frame = step(videoFileReader);
      [points, validity] = step(tracker, frame);
      out = insertMarker(frame, points(validity, :), '+');
      step(videoPlayer, out);
end