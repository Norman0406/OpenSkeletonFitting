#pragma once
#include "Logging.h"
#include <vector>

namespace osf
{
	class ModuleInputBase
	{
	public:
		ModuleInputBase(int index);
		~ModuleInputBase();

		int getIndex() const;

	private:
		int m_index;
	};

	template <typename T>
	class ModuleInput
		: public ModuleInputBase
	{
	public:
		ModuleInput(int index, const T*& data)
			: ModuleInputBase(index), m_data(data) {
				LOG << "new input [type, index]: \"" << typeid(T).name() << "\", " << index << ENDL;
		}
		~ModuleInput();

		const T*& getData() const {
			return m_data;
		}

	private:
		 const T*& m_data;
	};

	class ModuleInputList
	{
	public:
		ModuleInputList();
		~ModuleInputList();

		template <typename T>
		int addInput(const T*& data)
		{
			ModuleInput<T>* out = new ModuleInput<T>(m_list.size(), data);
			m_list.push_back(out);
			return out->getIndex();
		}

		template <typename T>
		ModuleInput<T>* getInput(int index)
		{
			if (index < 0 || index >= (int)m_list.size())
				throw Exception("index out of range");

			return static_cast<ModuleInput<T>*>(m_list[index]);
		}

	private:
		std::vector<ModuleInputBase*> m_list;
	};
}