/* Pure bounded census; no payload dereferences, allocation, or native writes. */
#ifndef DG_COMMAND_CENSUS_H
#define DG_COMMAND_CENSUS_H
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#define CQ_LIMIT 65536
#define CQ_HASH 131072
typedef int (*CQ_READ)(void*,uint64_t,void*,unsigned);
typedef struct {uint64_t address,before[3],after[3];} CQ_NODE;
typedef struct {
 uint64_t start,end,cursor,head,tail;unsigned count,changed;int error;
 CQ_NODE nodes[CQ_LIMIT];uint64_t seen[CQ_HASH];
} CQ_SAMPLE;
enum {CQ_OK,CQ_RANGE,CQ_READ_FAIL,CQ_CYCLE,CQ_OVERFLOW,CQ_ARENA_CHANGED};





























#endif
