#ifndef RC_FILTER__H_
#define RC_FILTER__H_

#ifdef __cplusplus
extern "C" {
#endif

#include "filter_base.h"
#include "stdio.h"

#define RC_PARAM_SIZE 1
#define RC_CUTOFF 0

#define RC_MEMORY_SIZE 1

// void rc_init(DTYPE cutoff);

void rc_update(DTYPE now, FT *filter);

// void rc_get_last_input(FT *filter, int i);

#ifdef __cplusplus
}
#endif

#endif