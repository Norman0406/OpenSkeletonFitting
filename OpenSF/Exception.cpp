#include "precompiled.h"
#include "Exception.h"
#include <iostream>

namespace osf
{
	Exception::Exception(std::string msg)
		: m_message(msg)
	{
		std::cerr << "Exception: " << m_message << std::endl;
	}

	Exception::~Exception(void) throw()
	{
	}

	const char* Exception::what() const throw()
	{
		return m_message.c_str();
	}
}
