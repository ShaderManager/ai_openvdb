#pragma once

#include <string>
#include <vector>
#include <ai.h>

#include <openvdb/math/Vec3.h>

namespace str
{
	std::vector<std::string> split(const std::string& src, const char* delimeters);
}

template<typename T> T max_comp(const openvdb::math::Vec3<T> v)
{
	return std::max(v.x(), std::max(v.y(), v.z()));
}

struct AtStringKeyHasher
{
	size_t operator ()(const AtString& key) const
	{
		return key.hash();
	}

	bool operator ()(const AtString& lhs, const AtString& rhs) const
	{
		return lhs == rhs;
	}
};

template<typename T> T read_user_param(const AtNode* node, const char* name, T default_value);
