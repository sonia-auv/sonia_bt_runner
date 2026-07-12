#include <cassert>
#include <cstdlib>
#include <cstring>

#include "sonia_bt_runner/utils/AUV.hpp"

namespace utils {
static int sub = SUB_NONE;

int get_auv()
{
	if (sub == SUB_NONE) {
		char *auv = getenv("AUV");
		
		if (auv) {
			if (!strcmp(auv, "AUV8")) {
				sub =  SUB_AUV8;
			} else if (!strcmp(auv, "LITE1")) {
				sub = SUB_LITE1;
			} else {
				assert(0 && "Unknow AUV");
			}
		} else {
			assert(0 && "Unknow AUV");
		}
	}

	return sub;
}
}
