#ifndef FILTER_BASE__H_
#define FILTER_BASE__H_

#ifdef __cplusplus
extern "C" {
#endif

typedef float DTYPE;

typedef struct Filter {
  DTYPE *last_input;
  DTYPE *last_output;
  DTYPE current_input;
  DTYPE current_output;
  int size;
  int last_pos;
  DTYPE last_time;
  DTYPE *param;
} FT;

#ifdef __cplusplus
}
#endif

#endif