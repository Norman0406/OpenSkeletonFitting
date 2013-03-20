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

#pragma once
#include "Logging.h"
#include <vector>

namespace osf
{
	class ModuleOutputBase
	{
	public:
		ModuleOutputBase(int index);
		~ModuleOutputBase();

		int getIndex() const;

	private:
		int m_index;
	};

	template <typename T>
	class ModuleOutput
		: public ModuleOutputBase
	{
	public:
		ModuleOutput(int index, T* data)
			: ModuleOutputBase(index) {
				LOG << "new output [type, index]: \"" << typeid(T).name() << "\", " << index << ENDL;
				m_data = data;
		}
		~ModuleOutput();

		T* getData() const {
			return m_data;
		}

	private:
		T* m_data;
	};

	class ModuleOutputList
	{
	public:
		ModuleOutputList();
		~ModuleOutputList();

		template <typename T>
		int addOutput(T* data)
		{
			ModuleOutput<T>* out = new ModuleOutput<T>(m_list.size(), data);
			m_list.push_back(out);
			return out->getIndex();
		}

		template <typename T>
		ModuleOutput<T>* getOutput(int index)
		{
			if (index < 0 || index >= (int)m_list.size())
				throw Exception("index out of range");

			return static_cast<ModuleOutput<T>*>(m_list[index]);
		}

	private:
		std::vector<ModuleOutputBase*> m_list;
	};
}