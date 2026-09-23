#ifndef DG_PAYLOAD_PROBE_H
#define DG_PAYLOAD_PROBE_H
#include "dg_command_census.h"
#define CP_LIMIT 4096
#define CP_BYTES (1024*1024)
typedef struct {uint64_t address;unsigned size,offset;} CP_RANGE;
typedef struct {unsigned count,bytes,changed;int error,complete;CP_RANGE ranges[CP_LIMIT];unsigned char before[CP_BYTES],after[CP_BYTES];} CP_SAMPLE;
enum {CP_OK,CP_RANGE_ERROR,CP_RANGE_LIMIT,CP_BYTE_LIMIT,CP_READ_ERROR,CP_LAYOUT_ERROR};


































/* Validate copied companion pointer, never dereference the game pointer. */
















#endif
