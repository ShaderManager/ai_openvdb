#include <stdlib.h>
#include <string.h>

#include <ai.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/ValueTransformer.h>

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

enum class GridType : int
{
	Unknown,
	Float,
};

struct Grid
{
	GridType type = GridType::Unknown;
	int32_t offset = -1;
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

	openvdb::GridBase::ConstPtr get_grid_ptr(const Grid& grid) const
	{
		switch (grid.type)
		{
		case GridType::Float:
			return float_grids[grid.offset];
		default:
			return openvdb::GridBase::ConstPtr();
		}
	}

	std::unordered_map<AtString, Grid, AtStringKeyHasher, AtStringKeyHasher> grids;

	std::vector<openvdb::FloatGrid::ConstPtr> float_grids;

	openvdb::FloatGrid::ConstPtr density_grid;
	openvdb::Vec3SGrid::Ptr source_velocity_grid;
	openvdb::Vec3SGrid::ConstPtr velocity_grid;

	float velocity_scale;
	float velocity_shutter_start = -0.25;
	float velocity_shutter_end = 0.25f;

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

		std::vector<std::string> grids_to_read;

		auto grid_reader = [&](const std::string& grid_name)
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
			else
			{
				return;
			}

			AiAddMemUsage(grid->memUsage(), "OpenVDB");			
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

		std::vector<std::string> velocity_grids
		{
			"VelocitiesX",
			"VelocitiesY",
			"VelocitiesZ"
		};

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
				}
				else if (openvdb::FloatGrid::gridType() == grid->type())
				{
					if (!new_volume->velocity_grid)
					{
						new_volume->source_velocity_grid = openvdb::Vec3SGrid::create(*grid);
						new_volume->velocity_grid = openvdb::gridConstPtrCast<openvdb::Vec3SGrid>(new_volume->source_velocity_grid);
					}

					auto comp_grid = openvdb::gridConstPtrCast<openvdb::FloatGrid>(grid);

					int comp = 0;

					if (grid_name.back() == 'Y' || grid_name.back() == 'y')
					{
						comp = 1;
					}
					else if (grid_name.back() == 'Z' || grid_name.back() == 'z')
					{
						comp = 2;
					}

					auto op = [comp]
					(const openvdb::FloatGrid::ValueOnCIter& it, openvdb::Vec3SGrid::Accessor& accessor)
					{
						if (it.isVoxelValue())
						{ // set a single voxel
							auto val = accessor.getValue(it.getCoord());

							switch (comp)
							{
							case 0:
								val.x() = it.getValue();
								break;
							case 1:
								val.y() = it.getValue();
								break;
							case 2:
								val.z() = it.getValue();
								break;
							}

							accessor.setValue(it.getCoord(), val);
						}
						else
						{ // fill an entire tile
							openvdb::math::CoordBBox bbox;
							it.getBoundingBox(bbox);
							accessor.getTree()->fill(bbox, openvdb::Vec3s(*it));
						}
					};

					openvdb::tools::transformValues(comp_grid->cbeginValueOn(), *new_volume->source_velocity_grid, op);
				}
			}
		}

		if (new_volume->density_grid)
		{
			new_volume->intersector = new openvdb::tools::VolumeRayIntersector<openvdb::FloatGrid>(*new_volume->density_grid, new_volume->total_bbox);
			volume->auto_step_size = new_volume->density_grid->voxelSize().x();
		}
		else
		{
			return false;
		}

		openvdb::Vec3s min_velocity(0, 0, 0);
		openvdb::Vec3s max_velocity(0, 0, 0);

		if (new_volume->velocity_grid)
		{
			new_volume->velocity_grid->evalMinMax(min_velocity, max_velocity);
			new_volume->velocity_scale = 1.f; // / 24.f;
		}

		for (auto& grid_it : new_volume->grids)
		{
			openvdb::GridBase::ConstPtr grid = new_volume->get_grid_ptr(grid_it.second);

			if (!grid)
				continue;

			auto grid_bbox = grid->evalActiveVoxelBoundingBox();
			
			auto world_grid_bbox = grid->transform().indexToWorld(grid_bbox);

			world_grid_bbox.expand(max_velocity * new_volume->velocity_scale);

			grid_bbox = grid->transform().worldToIndexNodeCentered(world_grid_bbox);
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

template<typename Result, typename GridType> Result sample_grid(int interpolation, const typename GridType::ConstUnsafeAccessor& accessor,
	const openvdb::math::Transform& transform, const openvdb::Vec3d sampling_point)
{
	using namespace openvdb::tools;

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

			value->FLT = sample_grid<float, openvdb::FloatGrid>(interpolation, accessor, grid_ptr->transform(), sampling_point);

			*type = AI_TYPE_FLOAT;
		}
		break;
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

		const float shutter_start = AiCameraGetShutterStart();
		const float shutter_end = AiCameraGetShutterEnd();

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

volume_plugin_loader
{
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
