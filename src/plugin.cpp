#include <stdlib.h>
#include <string.h>

#include <ai.h>

#include <openvdb/openvdb.h>

bool volume_init(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume)
{
	return false;
}

bool volume_update(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume)
{
	return false;
}

bool volume_finish(void* user_ptr, AtVolumeData* volume, const AtNode* node)
{
	return false;
}

bool volume_sample(void* user_ptr, const AtVolumeData* volume, const AtString channel, 
	const AtShaderGlobals* sg, int interpolation, 
	AtParamValue* value, AtByte* type)
{
	return false;
}

void volume_ray_extents(void* user_ptr, const AtVolumeData* volume, const AtVolumeIntersectionInfo* info, AtByte tid, float time, const AtPoint* origin, const AtVector* direction, float t0, float t1)
{

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
	vtable->UpdateVolume = volume_update;
	vtable->CleanupVolume = volume_finish;
	vtable->Sample = volume_sample;
	vtable->RayExtents = volume_ray_extents;
	return true;
}
