https://github.com/IntelRealSense/librealsense/issues/2688#issuecomment-436612914

The 400 Series has a pair of IR imagers that are used to create the depth image.

The baseline is between the left and right imagers. However, RealSense stereo cameras tend to rely on the left imager to generate the image. The dynamic calibration guide for the 400 Series says: "Assume left camera is the reference camera and is located at world origin."

The data sheet document for the 400 Series adds: "The left and right imagers capture the scene and sends imager data to the depth imaging processor, which calculates depth values for each pixel in the image by correlating points on the left image to the right image and via shift between a point on the Left image and the Right image".