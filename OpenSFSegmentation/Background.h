#pragma once
#include "../OpenSF/Module.h"

namespace osf
{
	class Background
		: public Module
	{
	public:
		Background(System*);
		virtual ~Background(void);
		
		virtual bool isInit() const;

	protected:
		virtual void iInit();
		virtual void iProcessBG() = 0;
		void iProcess();
		
		const cv::Mat* m_inImgDepth;
		cv::Mat m_bgModel;
	};
}
