#include <stdlib.h>
#include <string.h>

#include <ai.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/ValueTransformer.h>

#include <unordered_map>

#include "utils.hpp"

static const char* memory_category = "OpenVDB Plugin";

enum class GridType : int
{
	Unknown,
	Float,
	Vec3s
};

struct Grid
{
	GridType type = GridType::Unknown;
	int32_t offset = -1;
};

using FogVolumeRayIntersector = openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>;
using Ray = openvdb::math::Ray<openvdb::Real>;

struct Volume
{
	Volume()
	{

	}

	~Volume()
	{
		delete volume_intersector;
	}

	openvdb::GridBase::ConstPtr get_grid_ptr(const Grid& grid) const
	{
		assert(grid.offset >= 0);
		switch (grid.type)
		{
		case GridType::Float:
			assert(grid.offset < float_grids.size());
			return float_grids[grid.offset];
		case GridType::Vec3s:
			assert(grid.offset < vec3s_grids.size());
			return vec3s_grids[grid.offset];
		default:
			return openvdb::GridBase::ConstPtr();
		}
	}

	std::unordered_map<AtString, Grid, AtStringKeyHasher, AtStringKeyHasher> grids;

	std::vector<openvdb::FloatGrid::ConstPtr> float_grids;
	std::vector<openvdb::Vec3SGrid::ConstPtr> vec3s_grids;

	openvdb::FloatGrid::ConstPtr density_grid;
	openvdb::Vec3SGrid::Ptr converted_velocity_grid;
	openvdb::Vec3SGrid::ConstPtr velocity_grid;

	float velocity_scale = 0;
	float velocity_shutter_start = 0;
	float velocity_shutter_end = 0;

	FogVolumeRayIntersector* volume_intersector = nullptr;

	openvdb::math::CoordBBox total_bbox;

	std::string source_filename;
};

template<int comp> void upload_velocity_channel(openvdb::FloatGrid::ConstPtr comp_grid, openvdb::Vec3SGrid::Ptr velocity_grid)
{
	auto op = [](const openvdb::FloatGrid::ValueOnCIter& it, openvdb::Vec3SGrid::Accessor& accessor)
	{
		if (it.isVoxelValue()) // set a single voxel
		{
			const float value = it.getValue();
			accessor.modifyValue(it.getCoord(), [value](openvdb::Vec3s& val)
			{
				switch (comp)
				{
				case 0:
					val.x() = value;
					break;
				case 1:
					val.y() = value;
					break;
				case 2:
					val.z() = value;
					break;
				}
			});
		}
	};

	openvdb::tools::transformValues(comp_grid->cbeginValueOn(), *velocity_grid, op, false);	
}

bool volume_init(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume)
{
	auto new_volume = std::make_unique<Volume>();

	try
	{
		openvdb::io::File file(data);

		file.open(false);

		new_volume->source_filename = data;

		openvdb::BBoxd total_bbox;

		std::vector<std::string> grids_to_read = str::split(read_user_param(node, "grids", ""), " ,");

		auto grid_reader = [&](const std::string& grid_name) -> bool
		{
			auto grid = file.readGrid(grid_name);

			const AtString name(grid_name.c_str());

			if (grid->isType<openvdb::FloatGrid>())
			{
				new_volume->grids[name].type = GridType::Float;
				new_volume->grids[name].offset = new_volume->float_grids.size();
				new_volume->float_grids.push_back(openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid));

				if (!new_volume->density_grid)
				{
					new_volume->density_grid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid);
				}
			}
			else if (grid->isType<openvdb::Vec3SGrid>())
			{
				new_volume->grids[name].type = GridType::Vec3s;
				new_volume->grids[name].offset = new_volume->vec3s_grids.size();
				new_volume->vec3s_grids.push_back(openvdb::gridConstPtrCast<openvdb::Vec3SGrid>(grid));
			}
			else
			{
				AiMsgWarning("Unknown type '%s' of grid '%s'. Ignoring", grid->type().c_str(), grid_name.c_str());
				return false;
			}

			AiAddMemUsage(grid->memUsage(), memory_category);

			return true;
		};

		if (!grids_to_read.empty())
		{
			for (auto& grid_name : grids_to_read)
			{
				if (!file.hasGrid(grid_name))
				{
					AiMsgWarning("'%s' doesn't have grid with name '%s'", data, grid_name.c_str());
					continue;
				}

				grid_reader(grid_name);
			}
		}
		else
		{
			for (auto name_it = file.beginName(); name_it != file.endName(); ++name_it)
			{
				grid_reader(name_it.gridName());
			}
		}

		std::vector<std::string> velocity_grids = str::split(read_user_param(node, "velocity_grids", ""), " ,");;

		for (auto& grid_name : velocity_grids)
		{
			auto find_it = new_volume->grids.find(AtString(grid_name.c_str()));
			if (file.hasGrid(grid_name) || find_it != new_volume->grids.end())
			{
				openvdb::GridBase::ConstPtr grid;

				if (find_it == new_volume->grids.end())
				{
					grid = file.readGrid(grid_name);					
				}
				else
				{
					grid = new_volume->get_grid_ptr(find_it->second);
				}

				if (openvdb::Vec3SGrid::gridType() == grid->type())
				{
					new_volume->velocity_grid = openvdb::gridConstPtrCast<openvdb::Vec3SGrid>(grid);
					AiAddMemUsage(grid->memUsage(), memory_category);
				}
				else if (openvdb::FloatGrid::gridType() == grid->type())
				{	
					auto comp_grid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid);

					if (!new_volume->velocity_grid)
					{
						new_volume->converted_velocity_grid = openvdb::Vec3SGrid::create(openvdb::zeroVal<openvdb::Vec3s>());
						new_volume->converted_velocity_grid->setTransform(new_volume->density_grid->transform().copy());
						new_volume->velocity_grid = openvdb::gridConstPtrCast<openvdb::Vec3SGrid>(new_volume->converted_velocity_grid);
						AiAddMemUsage(new_volume->converted_velocity_grid->memUsage(), memory_category);
					}										

					if (grid_name.back() == 'X' || grid_name.back() == 'x')
					{
						upload_velocity_channel<0>(comp_grid, new_volume->converted_velocity_grid);
					}
					else if (grid_name.back() == 'Y' || grid_name.back() == 'y')
					{
						upload_velocity_channel<1>(comp_grid, new_volume->converted_velocity_grid);						
					}
					else // Z component
					{
						upload_velocity_channel<2>(comp_grid, new_volume->converted_velocity_grid);
					}
				}
			}
		}

		if (new_volume->density_grid)
		{
			switch (new_volume->density_grid->getGridClass())
			{
			case openvdb::GRID_LEVEL_SET:
				AiMsgError("Ray-marching through the level set is not supported!");
				return false;
				break;
			default:
				new_volume->volume_intersector = new FogVolumeRayIntersector(*new_volume->density_grid, new_volume->total_bbox);
				break;
			}

			volume->auto_step_size = max_comp(new_volume->density_grid->voxelSize());
		}
		else
		{
			AiMsgError("OpenVDB: There is no grid with type Float. Plugin can raytrace only through the volume of type float");
			return false;
		}

		openvdb::Vec3s min_velocity(0, 0, 0);
		openvdb::Vec3s max_velocity(0, 0, 0);

		if (new_volume->velocity_grid)
		{
			new_volume->velocity_grid->evalMinMax(min_velocity, max_velocity);

			new_volume->velocity_scale = read_user_param<float>(node, "velocity_scale", 1.f);
			new_volume->velocity_scale *= read_user_param<float>(node, "velocity_units", 1.f);
			new_volume->velocity_shutter_start = read_user_param<float>(node, "velocity_shutter_start", 0.f);
			new_volume->velocity_shutter_end = read_user_param<float>(node, "velocity_shutter_end", 0.f);
		}

		for (auto& grid_it : new_volume->grids)
		{
			openvdb::GridBase::ConstPtr grid = new_volume->get_grid_ptr(grid_it.second);

			if (!grid)
				continue;

			auto grid_bbox = grid->evalActiveVoxelBoundingBox();

			auto world_grid_bbox = grid->transform().indexToWorld(grid_bbox);

			if (new_volume->velocity_grid && new_volume->velocity_scale > 0)
			{
				world_grid_bbox.expand(max_velocity * new_volume->velocity_scale);

				grid_bbox = grid->transform().worldToIndexNodeCentered(world_grid_bbox);
			}

			new_volume->total_bbox.expand(grid_bbox);

			total_bbox.expand(world_grid_bbox);
		}

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
	return true;
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

template<typename GridType> auto sample_grid(int interpolation, const typename GridType::ConstUnsafeAccessor& accessor,
	const openvdb::math::Transform& transform, const openvdb::Vec3d sampling_point) -> typename GridType::ValueType
{
	using namespace openvdb::tools;
	using Result = GridType::ValueType;

	const auto index_point = transform.worldToIndex(sampling_point);
	Result result = openvdb::zeroVal<Result>();

	switch (interpolation)
	{
	case AI_VOLUME_INTERP_CLOSEST:
		PointSampler::sample(accessor, index_point, result);
		break;
	case AI_VOLUME_INTERP_TRILINEAR:
		BoxSampler::sample(accessor, index_point, result);
		break;
	case AI_VOLUME_INTERP_TRICUBIC:
		QuadraticSampler::sample(accessor, index_point, result);
		break;
	}

	return result;
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

	const auto& grid = grid_it->second;
	openvdb::Vec3d sampling_point(sg->Po.x, sg->Po.y, sg->Po.z);

	const float shutter_start = AiCameraGetShutterStart();
	const float shutter_end = AiCameraGetShutterEnd();

	if (shutter_start != shutter_end && self->velocity_grid)
	{
		auto accessor = self->velocity_grid->getConstUnsafeAccessor();

		auto velocity = accessor.getValue(self->velocity_grid->transform().worldToIndexCellCentered(sampling_point));

		const float rel_time = (sg->time - shutter_start) / (shutter_end - shutter_start);
		const float vel_time = LERP(rel_time, self->velocity_shutter_start, self->velocity_shutter_end);

		sampling_point = sampling_point - velocity * vel_time * self->velocity_scale;
	}

	switch (grid.type)
	{
	case GridType::Float:
	{
		const auto& grid_ptr = self->float_grids[grid.offset];
		auto accessor = grid_ptr->getConstUnsafeAccessor();

		value->FLT = sample_grid<openvdb::FloatGrid>(interpolation, accessor, grid_ptr->transform(), sampling_point);

		*type = AI_TYPE_FLOAT;
	}
	break;
	case GridType::Vec3s:
	{
		const auto& grid_ptr = self->vec3s_grids[grid.offset];

		auto accessor = grid_ptr->getConstUnsafeAccessor();

		const auto temp = sample_grid<openvdb::Vec3SGrid>(interpolation, accessor, grid_ptr->transform(), sampling_point);

		value->VEC.x = temp.x();
		value->VEC.y = temp.y();
		value->VEC.z = temp.z();

		*type = AI_TYPE_VECTOR;
	}
	break;
	}

	return true;
}

template<typename Intersector> void march_extents(const AtVolumeIntersectionInfo* info, const Ray& ray, const Intersector* master_intersector)
{
	Intersector intersector(*master_intersector);

	if (intersector.setWorldRay(ray))
	{
		Intersector::RealType t_min, t_max;
		while (intersector.march(t_min, t_max))
		{
			const auto near_pos = intersector.getWorldPos(t_min);
			const auto far_pos = intersector.getWorldPos(t_max);

			t_min = (near_pos - ray.eye()).length();
			t_max = (far_pos - ray.eye()).length();

			AiVolumeAddIntersection(info, t_min, t_max);
		}
	}
}

void volume_ray_extents(void* user_ptr, const AtVolumeData* volume, const AtVolumeIntersectionInfo* info, AtByte tid, float time, const AtPoint* origin, const AtVector* direction, float t0, float t1)
{
	auto self = static_cast<const Volume*>(volume->private_info);

	if (self && self->volume_intersector)
	{
		const openvdb::Vec3s ray_origin(origin->x, origin->y, origin->z);
		const openvdb::Vec3s ray_direction(direction->x, direction->y, direction->z);

		Ray ray(ray_origin, ray_direction, t0, t1);

		march_extents<FogVolumeRayIntersector>(info, ray, self->volume_intersector);
	}
}
