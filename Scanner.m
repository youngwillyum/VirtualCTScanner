% Virtual CT Scanner

% Define parameters

matrixSize = 256; % Matrix size for image reconstruction

numDetectors = 200; % Number of detectors in the scanner

detectorDistance = 2; % Distance between detectors in cm

sourceDistance = 5; % Distance between source and center of scanner in cm

rotationStepAngle = 1.4; % Step angle of detector rotation in degrees

% Generate test/validation phantom

phantomType = 'circular'; % Options: 'circular', 'rectangular'

phantomSize = 200; % Size of the phantom in cm

circleStruct1SI = 0.5; % Value for circular structures
circleStruct2SI = 0.2
circleStruct3SI = 0.8
circleStruct4SI = 0.6

rectangleSI = 0.5; % Value for rectangular structure

if strcmp(phantomType, 'circular')

    phantom = generateCircularPhantom(matrixSize, phantomSize, circleStruct1SI, circleStruct2SI, circleStruct3SI, circleStruct4SI);

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

% Display results

figure;

subplot(2,3,1);
imshow(phantom);
title('Original Phantom');

subplot(2,3,2);
imshow(reconstructedImage);
title('Reconstructed Image');

subplot(2,3,3);
imshow(imageDifference);
title('Image Difference');

subplot(2,3,[4 5]);
plot(signalIntensityProfiles(:,1), 'r');
hold on;
plot(signalIntensityProfiles(:,2), 'b');
xlabel('Position');
ylabel('Signal Intensity');
legend('Original Phantom', 'Reconstructed Image');
title('Signal Intensity Profiles');

% Function to generate circular phantom

function phantom = generateCircularPhantom(matrixSize, phantomSize, value1, value2, value3, value4)

    [x,y] = meshgrid(-matrixSize/2:matrixSize/2-1,-matrixSize/2:matrixSize/2-1);

    circle = sqrt(x.^2 + y.^2) < 100;

    shrunk = ~conv2(double(~circle), ones(3), 'same');

    % Create mask with circle structures with different centers and SI values
    centerOffsetsX = [-150 -100 -180 -120];
    centerOffsetsY = [-120 -100 -180 -160];

    circleStructsSI = [value1 value2 value3 value4];

    mask = zeros(size(circle));
    for i=1:numel(centerOffsetsX)
        centerX = matrixSize/2 + centerOffsetsX(i);
        centerY = matrixSize/2 + centerOffsetsY(i);
        mask = mask + (sqrt((x-centerX).^2 + (y-centerY).^2) < 20) * circleStructsSI(i);
    end

    phantom = (circle - shrunk) + mask;

end

% Function to generate rectangular phantom

function phantom = generateRectangularPhantom(matrixSize, phantomSize, value)

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
        if rotatedX < 0
            rotatedX = rotatedX + matrixSize;
        end

        rotatedY = x*sind(detectorAngle) + y*cosd(detectorAngle);
        if rotatedY < 0
            rotatedY = rotatedY + matrixSize;
        end

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

            pixelValue = -log(rayAttenuation(j) + eps);

            interpolatedPixelValue = interp2(reconstructedImage, rotatedX, rotatedY);

            if ~isnan(interpolatedPixelValue)
                reconstructedImage(round(rotatedY), round(rotatedX)) = interpolatedPixelValue + pixelValue;
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
