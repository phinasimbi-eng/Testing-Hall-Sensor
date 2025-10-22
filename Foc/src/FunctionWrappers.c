/*****************************************************************************
 * FOC Function Wrappers for GCC Build
 * Maps assembly-optimized function names to C implementations
 *****************************************************************************/

#include <stdint.h>
#include <string.h>

// Forward declarations from PublicDefine.c
extern int16_t Sinlt_c(int16_t Angle);
extern int16_t Coslt_c(int16_t Angle);

// Map function names to C implementations
int16_t Sinlt(int16_t Angle)
{
    return Sinlt_c(Angle);
}

int16_t Coslt(int16_t Angle)
{
    return Coslt_c(Angle);
}

// 32-bit limiting/saturation function
int32_t LLimit(int32_t Data, int32_t Min, int32_t Max)
{
    if (Data > Max)
        return Max;
    if (Data < Min)
        return Min;
    return Data;
}

// 16-bit limiting/saturation function
int16_t Limit(int16_t Data, int16_t Min, int16_t Max)
{
    if (Data > Max)
        return Max;
    if (Data < Min)
        return Min;
    return Data;
}

// SubCharFill - character fill function for OLED display
void SubCharFill(uint8_t *dest, uint8_t value, uint32_t count)
{
    memset(dest, value, count);
}
