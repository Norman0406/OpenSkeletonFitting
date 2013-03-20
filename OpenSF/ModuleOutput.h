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