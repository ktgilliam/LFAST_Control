#include <cinttypes>

typedef union
{
    uint32_t U32;
    int32_t I32;
    union
    {
        uint8_t U8[4];
        uint16_t U16[2];
        struct
        {
            uint16_t LOWER;
            uint16_t UPPER;
        } UNSIGNED;

        int16_t I16[2];
        struct
        {
            int16_t LOWER;
            int16_t UPPER;
        } SIGNED;

    } PARTS;
} ConversionBuffer32;

#define M32_LOWER 0x0000FFFF
#define M32_UPPER 0xFFFF0000