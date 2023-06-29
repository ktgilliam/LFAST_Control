#pragma once
#include <cinttypes>

typedef union
{
    uint32_t U32;
    int32_t I32;
    uint16_t U16[2];
    union
    {
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


template <typename T>
union ConversionBuffer
{
    T WHOLE;
    uint16_t U16_PARTS[sizeof(T)/sizeof(uint16_t)];
    uint8_t U8_PARTS[sizeof(T)/sizeof(uint8_t)];
};

template <typename T>
using ConversionBuffer_t = union ConversionBuffer<T>;


#define M32_LOWER 0x0000FFFF
#define M32_UPPER 0xFFFF0000