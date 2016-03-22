#pragma once

bool volume_init(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume);
bool volume_update(void* user_ptr, const char* data, const AtNode* node, AtVolumeData* volume);
bool volume_finish(void* user_ptr, AtVolumeData* volume, const AtNode* node);

bool volume_sample(void* user_ptr, const AtVolumeData* volume, const AtString channel,
	const AtShaderGlobals* sg, int interpolation,
	AtParamValue* value, AtByte* type);

void volume_ray_extents(void* user_ptr, const AtVolumeData* volume, const AtVolumeIntersectionInfo* info, AtByte tid, float time, const AtPoint* origin, const AtVector* direction, float t0, float t1);
