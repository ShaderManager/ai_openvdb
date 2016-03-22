#include "utils.hpp"

std::vector<std::string> str::split(const std::string& src, const char* delimeters)
{	
	std::string::size_type offset = 0;
	std::vector<std::string> result;

	if (src.empty())
	{
		return result;
	}

	do
	{
		const auto new_offset = src.find_first_of(delimeters, offset);

		if (new_offset != std::string::npos)
		{
			result.push_back(std::move(src.substr(offset, new_offset - offset)));
			offset = new_offset + 1;
			continue;
		}

		result.push_back(std::move(src.substr(offset)));
		offset = new_offset;

	} while (offset != std::string::npos);

	return result;
}

template<> float read_user_param(const AtNode* node, const char* name, float default_value)
{
	if (AiNodeLookUpUserParameter(node, name))
	{
		return AiNodeGetFlt(node, name);
	}

	return default_value;
}

template<> int read_user_param(const AtNode* node, const char* name, int default_value)
{
	if (AiNodeLookUpUserParameter(node, name))
	{
		return AiNodeGetInt(node, name);
	}

	return default_value;
}

template<> const char* read_user_param(const AtNode* node, const char* name, const char* default_value)
{
	if (AiNodeLookUpUserParameter(node, name))
	{
		return AiNodeGetStr(node, name);
	}

	return default_value;
}
