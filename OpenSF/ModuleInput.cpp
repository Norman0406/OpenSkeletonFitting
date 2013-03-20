#include "precompiled.h"
#include "ModuleInput.h"

namespace osf
{
	/************************************************************************
	  InputBase
	*************************************************************************/
	ModuleInputBase::ModuleInputBase(int index)
		: m_index(index)
	{
	}

	ModuleInputBase::~ModuleInputBase()
	{
	}

	int ModuleInputBase::getIndex() const
	{
		return m_index;
	}

	/************************************************************************
	  InputList
	*************************************************************************/
	ModuleInputList::ModuleInputList()
	{
	}

	ModuleInputList::~ModuleInputList()
	{
		for (std::vector<ModuleInputBase*>::const_iterator it = m_list.begin(); it != m_list.end(); it++)
			delete *it;
		m_list.clear();
	}
}
