# OpenSkeletonFitting

Skeleton fitting algorithm by the use of energy minimization

# Description

This is OpenSkeletonFitting (OpenSF). Is has been initiated as a research project at Osaka University, Japan from October 2011 to March 2012. It provides algorithms to enable skeleton tracking, using a single depth camera like Microsoft Kinect or Asus Xtion.

If you have ideas, constructive criticism or want to contribute to the project, please feel free to contact the author at anytime. Using this project or parts of it in derived works is highly encouraged. Please respect the licensing and mention the author "Norman Link <norman.link@gmx.net>".

You can find further information in the "Doc\" directory. Please note that these files are a bit outdated, but they can still give a good overview of the project.

# Modules

OpenSkeletonFitting (OpenSF) consists of the following modules:

## OpenSF

is the main framework. It consists of all necessary modules and utilities, which are needed by the modules

## OpenSFInput

delivers the input data from a connected sensor. It can create input data from Microsoft Kinect, a .oni file and a numbered image stream.

## OpenSFSegmentation

retrieves the input frames from the sensor and segment it, such that the background is removed and the user can be detected. This currently only supports a simple background subtraction, in which the first captured frame is considered to show the static background and all the following frames are subtracted against this background. Only one single user can be detected.

## OpenSFFeatures

uses the detected user silhouette and the corresponding depth pixels to reconstruct the point cloud and detect features that correspond to user end-affectors, i.e. hand, head, feet, torso. The end-affectors are detected by building a "geodesic distance map" of the detected user's point cloud, in which every pixel stores it's shortest distance from the user's center-of-mass. To be realtime-capable, a modified dijkstra's algorithm is used. The resulting distance map is then subsampled into "geodesic iso-patches", which are regions in the distance map of approximately the same distance from the center-of-mass. These iso-patches are then used to detect extrema in the distance map, which are assumed to correspond to end-affectors. "Geodesic lines" are then created by retrieving the geodesic path on the distance map from the end-affector to the center-of-mass.

This module also incorporates a simple feature tracking algorithm to create a trajectory for detected features and smooth the values using kalman filtering.

## OpenSFitting

defines a skeleton hierarchy consisting fo joints, bones and constraints. An energy function is then attached to each joint which uses the previously detected features, geodesic lines and the point cloud to define an energy, based on the joint parameters. The energy functions for the whole skeleton hierarchy are supposed to be minimal when the pose is optimal. A simple gradient minimization algorithm then uses the previous pose to minimize the overall skeleton energy.

A joint also defines classificator functions, which label the features to corresponding body parts, as well as extrapolator functions, which define the behaviour in case the body part has not been found.

## OpenSFLib

is the main library to which other libraries can link against.

## OpenSFApp

is a simple application which does not provide any visual output but runs the algorithms.

## OpenSFVis

visualizes the tracking result in either 2d or 3d representation using OpenGL.

# Build instructions

The library has been successfully tested to build on Microsoft Windows and MacOS X. For sake of simplicity, all required externals except boost are included in "Externals\".

## On Windows:

- Install boost 1.44 (you might as well try with later versions)
- define environment variable "BOOST_DIR" to point to the boost installation directory
- Open OpenSkeletonFitting with Visual Studio 2010
- build

## On MacOS X.

- Install boost libraries
- Create project files using CMake from CMakeLists.txt
- build

# License

This library is licensed under the [GNU General Public License](http://www.gnu.org/licenses/gpl-3.0.en.html, "GPLv3").