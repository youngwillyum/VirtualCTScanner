function ScannerF(handles)
% Virtual CT Scanner

% Define parameters
matrixSize = round(get(handles.MatrixSize, 'Value')); % Matrix size for image reconstruction
numDetectors = round(get(handles.NumOfDetectors, 'Value')); % Number of detectors in the scanner
detectorDistance = get(handles.DistanceOfDetectors, 'Value'); % Distance between detectors in cm
sourceDistance = get(handles.DistanceFromSource, 'Value'); % Distance between source and center of scanner in cm
rotationStepAngle = get(handles.AngleOfDetector, 'Value'); % Step angle of detector rotation in degrees

% Generate test/validation phantom
phantomType = 'circular'; % Options: 'circular', 'rectangular'
phantomSize = 200; % Size of the phantom in cm
circleStruct1SI = 0.5; % Value for circular structures
circleStruct2SI = 1.0;
circleStruct3SI = 0.8;
circleStruct4SI = 0.6;
rectangleSI = 0.5; % Value for rectangular structure

if strcmp(phantomType, 'rectangular')
    phantom = generateCircularPhantom(matrixSize, phantomSize, circleStruct4SI);
else
    phantom = generateRectangularPhantom(matrixSize, phantomSize, rectangleSI);
end

% Initialize sinogram array to store acquired data
sinogram = zeros(numDetectors, matrixSize);

% Perform data acquisition (simulate scanner operation)
for i=1:numDetectors
    
    detectorAngle = (i-1) * rotationStepAngle;
    
    % Calculate attenuation for each ray passing through the phantom
    rayAttenuation = calculateRayAttenuation(phantom, matrixSize, numDetectors, detectorDistance, sourceDistance, detectorAngle);
    
    % Store acquired data in sinogram array
    sinogram(i,:) = rayAttenuation;
    
end

% Perform image reconstruction using filtered backprojection algorithm
reconstructedImage = reconstructImage(sinogram, matrixSize, numDetectors, detectorDistance, sourceDistance, rotationStepAngle);

% Perform image analysis
signalIntensity = calculateSignalIntensity(reconstructedImage);
contrast = calculateContrast(signalIntensity);
imageDifference = calculateImageDifference(phantom, reconstructedImage);
signalIntensityProfiles = generateSignalIntensityProfiles(phantom, reconstructedImage);

% Display results on specific axes
imshow(phantom, 'Parent', handles.axes1);
title(handles.axes1, 'Original Phantom');

imshow(reconstructedImage, 'Parent', handles.axes2);
title(handles.axes2, 'Reconstructed Image');

imshow(imageDifference, 'Parent', handles.axes3);
title(handles.axes3, 'Image Difference');

% Plot on axes4
plot(handles.axes4, signalIntensityProfiles(:,1), 'r');
hold(handles.axes4, 'on');
plot(handles.axes4, signalIntensityProfiles(:,2), 'b');
xlabel(handles.axes4, 'Position');
ylabel(handles.axes4, 'Signal Intensity');
legend(handles.axes4, 'Original Phantom', 'Reconstructed Image');
title(handles.axes4, 'Signal Intensity Profiles');
hold(handles.axes4, 'off');

% Function to generate circular phantom
function phantom = generateCircularPhantom(matrixSize, phantomSize, value)
    [x,y] = meshgrid(-matrixSize/2:matrixSize/2-1,-matrixSize/2:matrixSize/2-1);
    circle = sqrt(x.^2 + y.^2) < 100;
    shrunk = ~conv2(double(~circle), ones(3), 'same');
    % TODO : MAKE MASK WITH CIRCLE STRUCTURES WITH DIFFERENT CENTERS AND
    % DIFFERENT SI VALUES
    phantom = circle - shrunk;
end

% Function to generate rectangular phantom
function phantom = generateRectangularPhantom(matrixSize, phantomSize,value)
    [x,y] = meshgrid(-matrixSize/2:matrixSize/2-1,-matrixSize/2:matrixSize/2-1);
    circle = sqrt(x.^2 + y.^2) < 100;
    shrunk = ~conv2(double(~circle), ones(3), 'same');
    mask = abs(x) < phantomSize/4 & abs(y) < phantomSize/4;
    phantom = (circle - shrunk) + value * mask;
end

% Function to calculate ray attenuation through the phantom
function rayAttenuation = calculateRayAttenuation(phantom,matrixSize,numDetectors,detectorDistance,sourceDistance,detectorAngle)
    rayAttenuation = zeros(1,matrixSize);
    for i=1:matrixSize
        x = (i-matrixSize/2-1) * detectorDistance;
        y = sourceDistance;
        rotatedX = x*cosd(detectorAngle) - y*sind(detectorAngle);
        rotatedY = x*sind(detectorAngle) + y*cosd(detectorAngle);
        pixelValue = interp2(phantom, rotatedX, rotatedY);
        rayAttenuation(i) = exp(-pixelValue);
    end
end

% Function to perform image reconstruction using filtered backprojection
function reconstructedImage = reconstructImage(sinogram,matrixSize,numDetectors,detectorDistance,sourceDistance,rotationStepAngle)
    reconstructedImage = zeros(matrixSize,matrixSize);
    for i=1:numDetectors
        detectorAngle = (i-1) * rotationStepAngle;
        rayAttenuation = sinogram(i,:);
        for j=1:matrixSize
            x = (j-matrixSize/2-1) * detectorDistance;
            y = sourceDistance;
            rotatedX = x*cosd(detectorAngle) - y*sind(detectorAngle);
            rotatedY = x*sind(detectorAngle) + y*cosd(detectorAngle);
            pixelValue = -log(rayAttenuation(j));
            interpolatedPixelValue = interp2(reconstructedImage, rotatedX, rotatedY);
            if ~isnan(interpolatedPixelValue)
                reconstructedImage(j,:) = interpolatedPixelValue + pixelValue;
            end
        end
    end
    reconstructedImage(isnan(reconstructedImage))=0;
end

% Function to calculate signal intensity in the reconstructed image
function signalIntensity = calculateSignalIntensity(reconstructedImage)
    signalIntensity.meanSI = mean(reconstructedImage(:));
    signalIntensity.maxSI = max(reconstructedImage(:));
    signalIntensity.minSI = min(reconstructedImage(:));
end

% Function to calculate contrast in the reconstructed image
function contrast = calculateContrast(signalIntensity)
    contrast.relativeContrast = (signalIntensity.maxSI - signalIntensity.minSI) / signalIntensity.meanSI;
end

% Function to calculate image difference between original phantom and reconstructed image
function imageDifference = calculateImageDifference(phantom, reconstructedImage)
    imageDifference = abs(phantom - reconstructedImage);
end

% Function to generate signal intensity profiles along specific directions in the reconstructed image
function signalIntensityProfiles = generateSignalIntensityProfiles(phantom, reconstructedImage)
    profile1 = phantom(:,round(size(phantom,2)/2));
    profile2 = reconstructedImage(:,round(size(reconstructedImage,2)/2));
    signalIntensityProfiles = [profile1 profile2];
end
end