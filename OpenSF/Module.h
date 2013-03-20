#pragma once
#include "Logging.h"
#include "Utils.h"
#include "ModuleInput.h"
#include "ModuleOutput.h"
#include "Timer.h"

#if __APPLE__ & __MACH__
#include <typeinfo>
using namespace std;
#endif

namespace osf
{

	// define a type string for factory creation
	#define MK_TYPE(classname) \
	public: static const type_info& getType() { return typeid(classname); } \
	private: \

	class System;
	class Timer;
	
	class Module
	{
	public:
		virtual ~Module(void);
	
		virtual void init();
		virtual bool isInit() const;
		float getLastTime() const;

		void process();
		
		template <typename T>
		ModuleOutput<T>* getOutput(int index)
		{
			return m_outputList.getOutput<T>(index);
		}

		template <typename T>
		T* getOutputData(int index)
		{
			return m_outputList.getOutput<T>(index)->getData();
		}

		template <typename T>
		ModuleInput<T>* getInput(int index)
		{
			return m_inputList.getInput<T>(index);
		}

		template <typename T>
		const T*& getInputData(int index)
		{
			return m_inputList.getInput<T>(index)->getData();
		}

	protected:
		Module(System*);

		virtual void iInit() = 0;
		virtual void iProcess() = 0;

		template <typename T>
		int addOutput(T* data)
		{
			return m_outputList.addOutput(data);
		}

		template <typename T>
		int addInput(const T*& data)
		{
			return m_inputList.addInput(data);
		}

		System* m_system;

	private:
		ModuleInputList		m_inputList;
		ModuleOutputList	m_outputList;
		Timer				m_timer;
		float				m_lastTime;
	};
}
