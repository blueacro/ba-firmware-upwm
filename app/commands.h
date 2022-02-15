#ifndef _COMMAND_H_
#define _COMMAND_H_

#include <stdint.h>
#include "utils.h"

enum COMMAND
{
    COMMAND_SET_OUTPUTS = 1,
    COMMAND_READ = 2,
    COMMAND_BOOTLOADER_ENTRY = 0xF0,
};

typedef struct PACK
{
    uint8_t command_byte;
} command_header_t;

typedef struct PACK
{
    command_header_t header;
    // No other command, all values are read
} command_read_t;

typedef struct PACK
{
    command_header_t header;
    uint32_t period;
    uint32_t out1;
    uint32_t out2;
    uint32_t out3;
    uint32_t out4;
} command_set_outputs_t;

////////////////////////////////
// Responses

enum RESPONSE
{
    RESPONSE_STATUS = 0,
};

typedef struct PACK
{
    uint8_t response_byte;
} response_header_t;

typedef struct PACK
{
    response_header_t header;
    uint32_t period;
    uint32_t out1;
    uint32_t out2;
    uint32_t out3;
    uint32_t out4;
} response_t;

#endif
