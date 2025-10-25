#include "mathFun.h"

void ChangeDataByte(uint8_t *p1, uint8_t *p2)
{
    uint8_t t;
    t = *p1;
    *p1 = *p2;
    *p2 = t;
}

void buffer_append_int32(uint8_t *buffer, int32_t source, int32_t *index)
{
    buffer[(*index)++] = source >> 24;
    buffer[(*index)++] = source >> 16;
    buffer[(*index)++] = source >> 8;
    buffer[(*index)++] = source;
}

/**
 * @brief Get the s32 data from four uint8_t buffer object
 * @param  buffer           My Param doc
 * @param  index            My Param doc
 * @return int32_t
 */
int32_t get_s32_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int32_t res = (((uint32_t)buffer[*index]) << 24) |
                  (((uint32_t)buffer[*index + 1]) << 16) |
                  (((uint32_t)buffer[*index + 2]) << 8) |
                  (((uint32_t)buffer[*index + 3]));
    *index += 4;
    return res;
}

/**
 * @brief transfer four int32_t type to float,??????
 * @param  buffer           My Param doc
 * @param  scale            My Param doc
 * @param  index            My Param doc
 * @return float
 */
float buffer_32_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s32_from_buffer(buffer, index) / scale;
}

/**
 * @brief Get the s16 data from two uint8_t buffer object
 * @param  buffer           My Param doc
 * @param  index            My Param doc
 * @return int16_t
 */
int16_t get_s16_from_buffer(const uint8_t *buffer, int32_t *index)
{
    int16_t res = (((uint32_t)buffer[*index]) << 8) |
                  (((uint32_t)buffer[*index + 1]));
    *index += 2;
    return res;
}

/**
 * @brief transfer four int16_t type to float
 * @param  buffer           My Param doc
 * @param  scale            My Param doc
 * @param  index            My Param doc
 * @return float
 */
float buffer_16_to_float(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)get_s16_from_buffer(buffer, index) / scale;
}

double cvtFloat2Double(float n1, float n2)
{
	struct {float n1;float n2;} s;
	s.n1 = n1;
	s.n2 = n2;
	return *((double*)&s);
}
