#pragma once
#include "../OpenSF/Module.h"
#include "../OpenSF/Factory.h"
#include "../OpenSF/Logging.h"

#if __APPLE__ & __MACH__
#include <typeinfo>
using namespace std;
#endif

namespace osf
{
	class Input;
	class Segmentation;
	class Features;
	class Fitting;

	class System
	{
	public:
		System();
		~System();
		
		void init();

		Input* createInput(const type_info&);
		Segmentation* createSegmentation(const type_info&);
		Features* createFeatures(const type_info&);
		Fitting* createFitting(const type_info&);
		bool getTerminate() const;
		void terminate(bool);

		const Input* getInput() const;
		const Segmentation* getSegmentation() const;
		const Features* getFeatures() const;
		const Fitting* getFitting() const;
		
		bool isInit() const;
		
		void prepare();
		void process();

	private:
		Input* m_input;
		Segmentation* m_segmentation;
		Features* m_features;
		Fitting* m_fitting;
		bool m_terminate;
	};
}
