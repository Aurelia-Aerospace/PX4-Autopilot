#pragma once
#include "param.h"
#include <cstring>

// @SECURE: always reject PARAM_SET/NSH param set — must use SecureCommand op 8
// @LOCKED: rejected when FW_LOCK == 1
// Only enforced on PX4_CRYPTO builds (ODID boards)

inline bool fw_param_is_secure(const char *name)
{
#ifdef PX4_CRYPTO
	static constexpr const char *list[] = {"FW_LOCK", "FW_SN", nullptr};
	for (auto p = list; *p; ++p) { if (strcmp(name, *p) == 0) { return true; } }
#endif
	return false;
}

inline bool fw_param_is_locked(const char *name)
{
#ifdef PX4_CRYPTO
	static param_t fw_lock = PARAM_INVALID;
	if (fw_lock == PARAM_INVALID) { fw_lock = param_find("FW_LOCK"); }
	if (fw_lock == PARAM_INVALID) { return false; }
	int32_t locked = 0;
	param_get(fw_lock, &locked);
	if (!locked) { return false; }
	static constexpr const char *list[] = {"COM_ARM_ODID", "UAVCAN_ENABLE", nullptr};
	for (auto p = list; *p; ++p) { if (strcmp(name, *p) == 0) { return true; } }
#endif
	return false;
}
