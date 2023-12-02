function GenerateBasicPhantomF(handles)
% Generate circular phantom
radius = get(handles.RadiusEditField, 'Value');
circleValue = get(handles.CircleSlider, 'Value');
rectValue = get(handles.RectangleSlider, 'Value');
phantomSize = round(get(handles.PhantomMatrixSize, 'Value'));

phantom1 = zeros(phantomSize);
center = [ceil(phantomSize/2), ceil(phantomSize/2)]; % Center coordinates

for i = 1:phantomSize
    for j = 1:phantomSize
        distance = sqrt((i - center(1))^2 + (j - center(2))^2);
        if distance <= radius
            phantom1(i, j) = circleValue; % Assign a value to circular structure
        end
    end
end


imshow(phantom1, 'Parent', handles.axes6);
title(handles.axes6,'Circular Phantom');


% Generate rectangular phantom
rectWidth = get(handles.WidthSlider, 'Value'); % Width of rectangular structure
rectHeight = get(handles.HeightSlider, 'Value'); % Height of rectangular structure

rectXStart = round((phantomSize - rectWidth)/2);
rectXEnd = rectXStart + round(rectWidth) - 1;
rectYStart = round((phantomSize - rectHeight)/2);
rectYEnd = rectYStart + round(rectHeight) - 1;

phantom2 = zeros(phantomSize);
phantom2(rectYStart:rectYEnd, rectXStart:rectXEnd) = rectValue; % Assign a value to rectangular structure


imshow(phantom2, 'Parent', handles.axes5);
title(handles.axes5, 'Rectangular Phantom');
