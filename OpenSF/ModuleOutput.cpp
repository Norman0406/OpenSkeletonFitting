#include "precompiled.h"
#include "ModuleOutput.h"

namespace osf
{
	/************************************************************************
	  OutputBase
	*************************************************************************/
	ModuleOutputBase::ModuleOutputBase(int index)
		: m_index(index)
	{
	}

	ModuleOutputBase::~ModuleOutputBase()
	{
	}

	int ModuleOutputBase::getIndex() const
	{
		return m_index;
	}

	/************************************************************************
	  OutputList
	*************************************************************************/
	ModuleOutputList::ModuleOutputList()
	{
	}

	ModuleOutputList::~ModuleOutputList()
	{
		for (std::vector<ModuleOutputBase*>::const_iterator it = m_list.begin(); it != m_list.end(); it++)
			delete *it;
		m_list.clear();
	}
}
