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

#include "precompiled.h"
#include "System.h"
#include "../OpenSFInput/Input.h"
#include "../OpenSFSegmentation/Segmentation.h"
#include "../OpenSFFeatures/Features.h"
#include "../OpenSFFeatures/FeaturePoint.h"
#include "../OpenSFitting/Fitting.h"

namespace osf
{
	System::System()
		: m_input(0), m_segmentation(0), m_features(0), m_fitting(0),
		m_terminate(false)
	{
	}

	System::~System()
	{
		m_input->shutdown();
		delete m_input;
		delete m_segmentation;
		delete m_features;
		delete m_fitting;
	}
	
	void System::init()
	{
		LOG << "init system" << ENDL;

		if (isInit())
			throw Exception("already init");

		// TODO: to be called before any creation
	}
	
	Input* System::createInput(const type_info& info)
	{
		m_input = Factory<Input>::createByType(info, this);
		return m_input;
	}
	
	Segmentation* System::createSegmentation(const type_info& info)
	{
		m_segmentation = Factory<Segmentation>::createByType(info, this);
		return m_segmentation;
	}

	Features* System::createFeatures(const type_info& info)
	{
		m_features = Factory<Features>::createByType(info, this);
		return m_features;
	}

	Fitting* System::createFitting(const type_info& info)
	{
		m_fitting = Factory<Fitting>::createByType(info, this);
		return m_fitting;
	}
	
	bool System::getTerminate() const
	{
		return m_terminate;
	}

	void System::terminate(bool term)
	{
		m_terminate = term;
	}

	const Input* System::getInput() const
	{
		return m_input;
	}

	const Segmentation* System::getSegmentation() const
	{
		return m_segmentation;
	}

	const Features* System::getFeatures() const
	{
		return m_features;
	}

	const Fitting* System::getFitting() const
	{
		return m_fitting;
	}

	bool System::isInit() const
	{
		return m_input && m_input->isInit() &&
			(m_segmentation ? m_segmentation->isInit() : true) &&
			(m_features ? m_features->isInit() : true) &&
			(m_fitting ? m_fitting->isInit() : true) &&
			true;
	}

	void System::prepare()
	{
		if (isInit())
			throw Exception("already init");

		m_input->init();
		
		if (m_segmentation) {
			// assign data to segmentation
			ModuleOutput<cv::Mat>* outInputDepth = m_input->getOutput<cv::Mat>(0);
			ModuleInput<cv::Mat>* inSegDepth = m_segmentation->getInput<cv::Mat>(0);
			inSegDepth->getData() = outInputDepth->getData();
		
			ModuleOutput<cv::Mat>* outInput3d = m_input->getOutput<cv::Mat>(2);
			ModuleInput<cv::Mat>* inSeg3d = m_segmentation->getInput<cv::Mat>(1);
			inSeg3d->getData() = outInput3d->getData();

			m_segmentation->init();
			
			if (m_features) {
				// assign data to feature detection
				ModuleOutput<cv::Mat>* outSegDepth = m_segmentation->getOutput<cv::Mat>(0);
				ModuleInput<cv::Mat>* inFeatDepth = m_features->getInput<cv::Mat>(0);
				inFeatDepth->getData() = outSegDepth->getData();
		
				ModuleOutput<cv::Mat>* outSeg3d = m_segmentation->getOutput<cv::Mat>(1);
				ModuleInput<cv::Mat>* inFeat3d = m_features->getInput<cv::Mat>(1);
				inFeat3d->getData() = outSeg3d->getData();

				m_features->init();

				if (m_fitting) {
					// assign data to skeleton fitting
					ModuleOutput<cv::Mat>* outFeatDepth = m_features->getOutput<cv::Mat>(0);
					ModuleInput<cv::Mat>* inFittingDepth = m_fitting->getInput<cv::Mat>(0);
					inFittingDepth->getData() = outFeatDepth->getData();
				
					ModuleOutput<cv::Mat>* outFeat3d = m_features->getOutput<cv::Mat>(1);
					ModuleInput<cv::Mat>* inFitting3d = m_fitting->getInput<cv::Mat>(1);
					inFitting3d->getData() = outFeat3d->getData();
		
					ModuleOutput<cv::Mat>* outFeatGeodesic = m_features->getOutput<cv::Mat>(2);
					ModuleInput<cv::Mat>* inFittingGeodesic = m_fitting->getInput<cv::Mat>(2);
					inFittingGeodesic->getData() = outFeatGeodesic->getData();

					ModuleOutput<cv::Mat>* outFeatPred = m_features->getOutput<cv::Mat>(3);
					ModuleInput<cv::Mat>* inFittingPred = m_fitting->getInput<cv::Mat>(3);
					inFittingPred->getData() = outFeatPred->getData();
					
					ModuleOutput<std::vector<FeaturePoint*> >* outFeatTrFt = m_features->getOutput<std::vector<FeaturePoint*> >(4);
					ModuleInput<std::vector<FeaturePoint*> >* inFittingTrFt = m_fitting->getInput<std::vector<FeaturePoint*> >(4);
					inFittingTrFt->getData() = outFeatTrFt->getData();

					m_fitting->init();
				}
			}
		}
	}

	void System::process()
	{
		if (!isInit())
			throw Exception("not init");

		std::stringstream stats;

		m_input->process();
		double timeInput = m_input->getLastTime();
		stats << "I: " << (int)timeInput;
		
		double overallProcessing = 0;
		if (m_segmentation) {
			m_segmentation->process();
			double timeSeg = m_segmentation->getLastTime();
			overallProcessing += timeSeg;
			stats << "\t S: " << (int)timeSeg;

			if (m_features) {
				m_features->process();
				double timeFeat = m_features->getLastTime();
				overallProcessing += timeFeat;
				stats << "\t F: " << (int)timeFeat;

				if (m_fitting) {
					m_fitting->process();
					double timeFit = m_fitting->getLastTime();
					overallProcessing += timeFit;
					stats << "\t Ft: " << (int)timeFit;
				}
			}
		}

		stats << "\t / " << overallProcessing;
		LOG << stats.str() << ENDL;
	}
}
