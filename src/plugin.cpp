#include <stdlib.h>
#include <string.h>

#include <ai.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/RayIntersector.h>

#include <unordered_map>
#include <string>

namespace str
{
	std::vector<std::string> split(const std::string& src, const char* delimeters)
	{
		std::string::size_type offset = 0;
		std::vector<std::string> result;

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

struct Volume
{
	Volume()
	{

	}

	~Volume()
	{
		delete intersector;
	}

	std::unordered_map<AtString, openvdb::GridBase::Ptr, AtStringKeyHasher, AtStringKeyHasher> grids;
	openvdb::FloatGrid::ConstPtr density_grid;
	openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>* intersector = nullptr;
	openvdb::math::CoordBBox total_bbox;
};

bool volume_init(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume)
{
	auto new_volume = std::make_unique<Volume>();

	try
	{
		openvdb::io::File file(data);

		file.open(false);

		openvdb::BBoxd total_bbox;

		for (auto name_it = file.beginName(); name_it != file.endName(); ++name_it)
		{
			auto grid = file.readGrid(name_it.gridName());

			new_volume->grids[AtString(name_it.gridName().c_str())] = grid;

			if (grid->isType<openvdb::FloatGrid>() && !new_volume->density_grid)
			{
				new_volume->density_grid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid);				
			}

			AiAddMemUsage(grid->memUsage(), "OpenVDB");

			const auto grid_bbox = grid->evalActiveVoxelBoundingBox();

			new_volume->total_bbox.expand(grid_bbox);
			total_bbox.expand(grid->transform().indexToWorld(grid_bbox));
		}

		if (new_volume->density_grid)
		{
			new_volume->intersector = new openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>(*new_volume->density_grid, new_volume->total_bbox);
		}

		volume->auto_step_size = new_volume->density_grid->voxelSize().x();

		volume->bbox.min.x = total_bbox.min().x();
		volume->bbox.min.y = total_bbox.min().y();
		volume->bbox.min.z = total_bbox.min().z();
		volume->bbox.max.x = total_bbox.max().x();
		volume->bbox.max.y = total_bbox.max().y();
		volume->bbox.max.z = total_bbox.max().z();

		volume->private_info = new_volume.release();
	}
	catch (openvdb::IoError& exc)
	{
		AiMsgError("VDB IO Error: %s", exc.what());
		return false;
	}

	return true;
}

bool volume_update(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume)
{
	return false;
}

bool volume_finish(void* user_ptr, AtVolumeData* volume, const AtNode* node)
{
	auto self = static_cast<Volume*>(volume->private_info);

	if (self)
	{
		delete self;

		return true;
	}

	return false;
}

template<typename Result, typename GridType> Result sample_grid(int interpolation, const typename GridType::ConstAccessor& accessor, 
	const openvdb::math::Transform& transform, const openvdb::Vec3d sampling_point)
{
	switch (interpolation)
	{
	case AI_VOLUME_INTERP_CLOSEST:
	{
		openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::PointSampler> sampler(accessor, transform);

		return sampler.wsSample(sampling_point);
	}
	break;
	case AI_VOLUME_INTERP_TRILINEAR:
	{
		openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, transform);

		return sampler.wsSample(sampling_point);
	}
	break;
	case AI_VOLUME_INTERP_TRICUBIC:
	{
		openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::QuadraticSampler> sampler(accessor, transform);

		return sampler.wsSample(sampling_point);
	}
	break;
	default:
		return Result();
	}
}

bool volume_sample(void* user_ptr, const AtVolumeData* volume, const AtString channel, 
	const AtShaderGlobals* sg, int interpolation, 
	AtParamValue* value, AtByte* type)
{
	auto self = static_cast<const Volume*>(volume->private_info);

	if (!self)
	{
		return false;
	}

	auto grid_it = self->grids.find(channel);

	if (grid_it == self->grids.end())
	{
		return false;
	}

	auto grid_ptr = grid_it->second;
	const openvdb::Vec3d sampling_point(sg->Po.x, sg->Po.y, sg->Po.z);

	if (grid_ptr->isType<openvdb::FloatGrid>())
	{
		auto grid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid_ptr);
		auto accessor = grid->getConstAccessor();		

		value->FLT = sample_grid<float, openvdb::FloatGrid>(interpolation, accessor, grid->transform(), sampling_point);

		*type = AI_TYPE_FLOAT;
	}

	return true;
}

void volume_ray_extents(void* user_ptr, const AtVolumeData* volume, const AtVolumeIntersectionInfo* info, AtByte tid, float time, const AtPoint* origin, const AtVector* direction, float t0, float t1)
{
	auto self = static_cast<const Volume*>(volume->private_info);

	if (self && self->density_grid)
	{
		openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid> intersector(*self->intersector);
		const openvdb::Vec3s ray_origin(origin->x, origin->y, origin->z);
		decltype(intersector)::RayType ray(ray_origin, openvdb::Vec3s(direction->x, direction->y, direction->z), t0, t1);

		if (intersector.setWorldRay(ray))
		{
			double t_min, t_max;
			while (intersector.march(t_min, t_max))
			{
				const auto near_pos = intersector.getWorldPos(t_min);
				const auto far_pos = intersector.getWorldPos(t_max);

				t_min = (near_pos - ray_origin).length();
				t_max = (far_pos - ray_origin).length();

				AiVolumeAddIntersection(info, t_min, t_max);
			}
		}
	}
}

bool plugin_init(void** user_ptr)
{
	openvdb::initialize();
	return true;
}

bool plugin_cleanup(void* user_ptr)
{
	openvdb::uninitialize();
	return true;
}

#include <windows.h>

volume_plugin_loader
{
	MessageBox(0, "!", "!", MB_OK);

	strcpy(vtable->version, AI_VERSION);
	vtable->Init = plugin_init;
	vtable->Cleanup = plugin_cleanup;
	vtable->CreateVolume = volume_init;
	//vtable->UpdateVolume = volume_update;
	vtable->CleanupVolume = volume_finish;
	vtable->Sample = volume_sample;
	vtable->RayExtents = volume_ray_extents;
	return true;
}
