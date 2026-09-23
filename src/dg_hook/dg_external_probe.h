/* Bounded read-only external graph snapshots. No allocation or output in capture. */
#ifndef DG_EXTERNAL_PROBE_H
#define DG_EXTERNAL_PROBE_H
#include "dg_payload_probe.h"
#define CE_LIMIT 4096
#define CE_BYTES (512*1024)
enum {CE_OK,CE_SOURCE,CE_ADDRESS,CE_LIMIT_ERROR,CE_READ,CE_LAYOUT};
enum {CE_OBJECT=1,CE_DESCRIPTOR,CE_LOOKUP,CE_RECORDS};
typedef struct {uint64_t address;unsigned size,offset,kind;} CE_RANGE;
typedef struct {unsigned count,bytes;int error,complete;CE_RANGE ranges[CE_LIMIT];unsigned char data[CE_BYTES];} CE_SAMPLE;



















































#endif
