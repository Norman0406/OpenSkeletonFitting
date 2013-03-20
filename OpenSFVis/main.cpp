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
#include "Visualization2d.h"
#include "Visualization3d.h"
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
		input->setMotorAngle(10);
		//input->startRecording("..\\..\\Data\\new_scene.oni");
		
		/*InputPlayerONI* input = dynamic_cast<InputPlayerONI*>(myOSF.createInput(InputPlayerONI::getType()));
		input->setFilename("..\\..\\Data\\scene03.oni");
		input->setRepeat(true);*/
		//input->setPauseMode(true);

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

		// create visualization objects
		Visualization2d vis2d(input, seg, feat, fitting);
		Visualization3d vis3d(input, seg, feat, fitting, cv::Size(800, 600));
		
		//vis2d.recordDepthMap("rec_depthmap.avi");
		//vis2d.recordSegmentation("rec_segmentation.avi");
		//vis2d.recordGeodesicMap("rec_geodesic.avi");
		//vis2d.recordSkeleton("rec_skeleton.avi", 0, 15);

		vis2d.init();
		vis3d.init();

		// main loop
		bool terminate = false;
		bool paused = false;
		bool step = false;
		do {
			// process
			if (!paused || step) {
				myOSF.process();
				step = false;
			}

			terminate |= myOSF.getTerminate();

			// draw 2d
			bool cvPaused = paused;
			bool cvStep = step;
			terminate |= vis2d.draw(cvPaused, cvStep);
			
			// draw 3d
			bool glPaused = paused;
			bool glStep = step;
			terminate |= vis3d.draw(glPaused, glStep);

			bool tempPaused = paused;
			if (cvPaused == !tempPaused)
				paused = cvPaused;
			if (glPaused == !tempPaused)
				paused = glPaused;
			
			bool tempStep = step;
			if (cvStep == !tempStep)
				step = cvStep;
			if (glStep == !tempStep)
				step = glStep;
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
