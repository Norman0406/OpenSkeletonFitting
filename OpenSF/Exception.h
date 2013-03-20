#pragma once
#include <exception>
#include <string>

namespace osf
{
	class Exception
		: std::exception
	{
	public:
		Exception(std::string);
		virtual ~Exception() throw();

		virtual const char* what() const throw();

	private:
		const std::string m_message;
	};
}
