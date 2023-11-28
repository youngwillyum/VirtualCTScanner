% Generate circular phantom
radius = 50; % Radius of circular structure
phantomSize = 256; % Size of the phantom matrix
phantom1 = zeros(phantomSize);
center = [ceil(phantomSize/2), ceil(phantomSize/2)]; % Center coordinates

for i = 1:phantomSize
    for j = 1:phantomSize
        distance = sqrt((i - center(1))^2 + (j - center(2))^2);
        if distance <= radius
            phantom1(i, j) = 100; % Assign a value to circular structure
        end
    end
end

figure;
imshow(phantom1, []);
title('Circular Phantom');

% Generate rectangular phantom
rectWidth = 80; % Width of rectangular structure
rectHeight = 120; % Height of rectangular structure

rectXStart = ceil((phantomSize - rectWidth)/2);
rectXEnd = rectXStart + rectWidth - 1;
rectYStart = ceil((phantomSize - rectHeight)/2);
rectYEnd = rectYStart + rectHeight - 1;

phantom2 = zeros(phantomSize);
phantom2(rectYStart:rectYEnd, rectXStart:rectXEnd) = 200; % Assign a value to rectangular structure

figure;
imshow(phantom2, []);
title('Rectangular Phantom');