/***********************************************************************
*
* OpenSkeletonFitting
* Skeleton fitting by the use of energy minimization
* Copyright (C) 2012 Norman Link <norman.link@gmx.net>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***********************************************************************/

#include <iostream>
#include "../OpenSF/Logging.h"
#include "../OpenSFInput/InputKinect.h"
#include "../OpenSFInput/InputNumbered.h"
#include "../OpenSFInput/InputPlayerONI.h"
#include "../OpenSFSegmentation/SegmentationBackground.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFitting/Fitting.h"
#include "../OpenSFitting/SkeletonHuman.h"
#include "../OpenSFitting/SkeletonManipulator.h"
#include "../OpenSFLib/System.h"
using namespace std;
using namespace osf;

int main()
{
	try {
		System myOSF;
		myOSF.init();
		
		// create input object
		InputKinect* input = dynamic_cast<InputKinect*>(myOSF.createInput(InputKinect::getType()));
		input->setRegisterDepth(true);

		// resize for better performance
		input->setResizing(cv::Size(320, 240));

		// create segmentation object
		Segmentation* seg = 0;
		seg = myOSF.createSegmentation(SegmentationBackground::getType());

		// create feature detection
		Features* feat = 0;
		feat = myOSF.createFeatures(Features::getType());

		// set parameters for feature detection
		if (feat) {
			feat->setGeoMaxZDistThreshold(0.1f);
			feat->setGeoNeighborPrecision(8);
			feat->setIsoPatchResizing(cv::Size(160, 120));
			feat->setIsoPatchWidth(0.2f);
			feat->setTrSearchRadius(0.3f);
			feat->setTrFtLifespan(10);
			feat->setTrFtPointTempTimespan(20);
			feat->setTrFtKfMeasurementNoise(1e-5);
			feat->setTrFtKfProcessNoise(1e-6);
		}
		
		// create skeleton fitting
		Fitting* fitting = 0;
		fitting = myOSF.createFitting(Fitting::getType());

		// set parameters for skeleton fitting
		if (fitting) {
			fitting->setNNDepthStep(3);
			fitting->setNNDepthMaxLeafSize(15);
			fitting->setNNGeoStep(1);
			fitting->setNNGeoMaxLeafSize(15);
			fitting->setNNGeoCutoffFactor(0.5f);
			fitting->setFitCCDMaxIter(1);
			fitting->setFitCCDChangeThresh(0.001);
			fitting->setFitCCDMinimzeSize(true);

			// select skeleton to track
			fitting->createSkeleton<SkeletonUpperBody>();
			//fitting->createSkeleton<SkeletonLowerBody>();
			//fitting->createSkeleton<SkeletonFullBody>();
			//fitting->createSkeleton<SkeletonSimple>();
			//fitting->createSkeleton<SkeletonManipulator>();
		}
		
		// init system
		myOSF.prepare();

		// main loop
		bool terminate = false;
		bool paused = false;
		do {
			// process
			if (!paused)
				myOSF.process();

			terminate |= myOSF.getTerminate();
		} while (!terminate);
	}
	catch (Exception& e) {
		ERR << "Exception: \"" <<
			e.what() << "\"" << ENDL;
	}
	catch (std::exception& e) {
		ERR << "std::exception: \"" <<
			e.what() << "\"" << ENDL;
	}
	catch (...) {
		ERR << "unknown exception" << ENDL;
	}

	return 0;
}

