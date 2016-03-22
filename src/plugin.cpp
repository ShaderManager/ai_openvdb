#include <ai.h>

#include <openvdb/openvdb.h>

#include "volume.hpp"

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
