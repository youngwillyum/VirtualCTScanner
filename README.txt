# Virtual CT Scanner

This MATLAB code simulates the operation of a virtual CT scanner. It performs various tasks such as generating a test/validation phantom, acquiring data, reconstructing an image using the filtered backprojection algorithm, performing image analysis, and displaying the results.
## How to run  

Open scanner.m in matlab, click the run button. 

## Parameters

The following parameters can be modified to customize the simulation:

- `matrixSize`: Matrix size for image reconstruction.
- `numDetectors`: Number of detectors in the scanner.
- `detectorDistance`: Distance between detectors in cm.
- `sourceDistance`: Distance between source and center of scanner in cm.
- `rotationStepAngle`: Step angle of detector rotation in degrees.
- `acquisitionTimePerRotation`: Duration of data acquisition per rotation in seconds.

## Generating Phantom

The code allows generating two types of phantoms: circular and rectangular. The size and intensity values of the phantoms can be adjusted.

## Data Acquisition

The code simulates the operation of a CT scanner by acquiring data from each detector as it rotates around the phantom. It calculates attenuation for each ray passing through the phantom and stores the acquired data in a sinogram array.

## Image Reconstruction

Using the acquired data (sinogram), the code performs image reconstruction using the filtered backprojection algorithm. The reconstructed image is stored in a variable called `reconstructedImage`.

## Image Analysis

The code performs various analyses on the reconstructed image, including calculating signal intensity, contrast, image difference between the original phantom and reconstructed image, and generating signal intensity profiles along specific directions in the reconstructed image.

## Results Display

Finally, the code displays several plots to visualize the results:

1. Original Phantom: Displays the original phantom used for simulation.
2. Reconstructed Image: Displays the reconstructed image after performing image reconstruction.
3. Image Difference: Shows the absolute difference between the original phantom and reconstructed image.
4. Signal Intensity Profiles: Plots signal intensity profiles along specific directions in the original phantom and reconstructed image.

Note: The code includes several helper functions to perform different tasks, such as generating phantoms, calculating ray attenuation, calculating signal intensity and contrast, etc.

Please refer to the code comments for more details on each function and its purpose.