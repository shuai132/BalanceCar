#ifndef _DISPATCH_H_
#define _DISPATCH_H_

#include <stdint.h>
#include <string.h>
#include "cube_hal.h"
#include "bluenrg_sdk_api.h"

typedef struct _operation_t operation_t;
typedef struct _dispatch_queue_t dispatch_queue_t;

struct _operation_t {
    operation_t* next;
    function_t func;
    void* args;
    uint32_t timestamp;
};

struct _dispatch_queue_t {
    operation_t *head, *tail;
};

void dispatch_init( void );
void dispatch( void );

#endif // _DISPATCH_H_



