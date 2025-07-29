/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "buffer.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>  // Für memset in VescUart.cpp

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/*
 * See my question:
 * http://stackoverflow.com/questions/40416682/portable-way-to-serialize-float-as-32-bit-integer
 *
 * Regarding the float32_auto functions:
 *
 * Noticed that frexp and ldexp fit the format of the IEEE float representation, so
 * they should be quite fast. They are (more or less) equivalent with the following:
 *
 * float frexp_slow(float f, int *e) {
 *     if (f == 0.0) {
 *         *e = 0;
 *         return 0.0;
 *     }
 *
 *     *e = ceilf(log2f(fabsf(f)));
 *     float res = f / powf(2.0, (float)*e);
 *
 *     if (res >= 1.0) {
 *         res -= 0.5;
 *         *e += 1;
 *     }
 *
 *     if (res <= -1.0) {
 *         res += 0.5;
 *         *e += 1;
 *     }
 *
 *     return res;
 * }
 *
 * float ldexp_slow(float f, int e) {
 *     return f * powf(2.0, (float)e);
 * }
 *
 * 8388608.0 is 2^23, which scales the result to fit within 23 bits if sig_abs < 1.0.
 *
 * This should be a relatively fast and efficient way to serialize
 * floating point numbers in a fully defined manner.
 */
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index) {
	// Set subnormal numbers to 0 as they are not handled properly
	// using this method.
	if (fabsf(number) < 1.5e-38) {
		number = 0.0;
	}

	int e = 0;
	float sig = frexpf(number, &e);
	float sig_abs = fabsf(sig);
	uint32_t sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e += 126;
	}

	uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1U << 31;
	}

	buffer_append_uint32(buffer, res, index);
}

// REPARIERTE BUFFER-GET-FUNKTIONEN mit Boundary-Checks
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	// SICHERHEITSCHECK: Prüfe Buffer-Grenzen und Null-Pointer
	if (!buffer || !index || (*index) < 0 || (*index) + 1 >= 1024) {
		if (index) *index = 1024; // Setze an sicheres Ende
		return 0;
	}
	
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
	// SICHERHEITSCHECK: Prüfe Buffer-Grenzen und Null-Pointer
	if (!buffer || !index || (*index) < 0 || (*index) + 1 >= 1024) {
		if (index) *index = 1024;
		return 0;
	}
	
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	// SICHERHEITSCHECK: Prüfe Buffer-Grenzen und Null-Pointer
	if (!buffer || !index || (*index) < 0 || (*index) + 3 >= 1024) {
		if (index) *index = 1024;
		return 0;
	}
	
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
	// SICHERHEITSCHECK: Prüfe Buffer-Grenzen und Null-Pointer
	if (!buffer || !index || (*index) < 0 || (*index) + 3 >= 1024) {
		if (index) *index = 1024;
		return 0;
	}
	
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int16(buffer, index) / scale;
}

// KRITISCHE FUNKTION: buffer_get_float32 - hier passiert der Stack Overflow!
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
	// ERWEITERTE SICHERHEITSCHECKS gegen Buffer Overflow
	if (!buffer || !index) {
		return 0.0f;
	}
	
	// Zusätzliche Validierung der scale
	if (scale == 0.0f || !isfinite(scale)) {
		if (index) *index += 4; // Skip bytes to maintain protocol
		return 0.0f;
	}
	
	// Sichere int32 Extraktion mit Boundary-Checks
    int32_t intValue = buffer_get_int32(buffer, index);
    float result = (float)intValue / scale;
    
    // Validierung des Ergebnisses
    if (!isfinite(result)) {
    	return 0.0f;
    }
    
    return result;
}

float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index) {
	// SICHERE uint32 Extraktion mit Boundary-Checks
	uint32_t res = buffer_get_uint32(buffer, index);

	int e = (res >> 23) & 0xFF;
	uint32_t sig_i = res & 0x7FFFFF;
	bool neg = res & (1U << 31);

	float sig = 0.0;
	if (e != 0 || sig_i != 0) {
		sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
		e -= 126;
	}

	if (neg) {
		sig = -sig;
	}

	float result = ldexpf(sig, e);
	
	// Validierung des Ergebnisses
	if (!isfinite(result)) {
		return 0.0f;
	}

	return result;
}


bool buffer_get_bool(const uint8_t *buffer, int32_t *index) {
	// SICHERHEITSCHECK: Prüfe Buffer-Grenzen und Null-Pointer
	if (!buffer || !index || (*index) < 0 || (*index) >= 1024) {
		if (index) (*index)++;
		return false;
	}
	
	if (buffer[*index] == 1) {
		(*index)++;
		return true;
	} else {
		(*index)++;
		return false;
	}
}

void buffer_append_bool(uint8_t *buffer,bool value, int32_t *index) {

	if (value == true)
	{
		buffer[*index] = 1;
		(*index)++;
	}
	else
	{
		buffer[*index] = 0;
		(*index)++;
	}

}

// NEUE EXPLIZIT SICHERE VERSIONEN mit Buffer-Längenkontrolle
int16_t buffer_get_int16_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	if (!buffer || !index || (*index) < 0 || (*index) + 1 >= buffer_len) {
		if (index) *index = buffer_len;
		return 0;
	}
	
	int16_t res = ((uint16_t) buffer[*index]) << 8 | ((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

uint16_t buffer_get_uint16_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	if (!buffer || !index || (*index) < 0 || (*index) + 1 >= buffer_len) {
		if (index) *index = buffer_len;
		return 0;
	}
	
	uint16_t res = ((uint16_t) buffer[*index]) << 8 | ((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t buffer_get_int32_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	if (!buffer || !index || (*index) < 0 || (*index) + 3 >= buffer_len) {
		if (index) *index = buffer_len;
		return 0;
	}
	
	int32_t res = ((uint32_t) buffer[*index]) << 24 |
				  ((uint32_t) buffer[*index + 1]) << 16 |
				  ((uint32_t) buffer[*index + 2]) << 8 |
				  ((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

uint32_t buffer_get_uint32_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	if (!buffer || !index || (*index) < 0 || (*index) + 3 >= buffer_len) {
		if (index) *index = buffer_len;
		return 0;
	}
	
	uint32_t res = ((uint32_t) buffer[*index]) << 24 |
				   ((uint32_t) buffer[*index + 1]) << 16 |
				   ((uint32_t) buffer[*index + 2]) << 8 |
				   ((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

float buffer_get_float16_safe(const uint8_t *buffer, float scale, int32_t *index, int32_t buffer_len) {
	return (float)buffer_get_int16_safe(buffer, index, buffer_len) / scale;
}

float buffer_get_float32_safe(const uint8_t *buffer, float scale, int32_t *index, int32_t buffer_len) {
	if (scale == 0.0f || !isfinite(scale)) {
		if (index && (*index) + 3 < buffer_len) *index += 4;
		return 0.0f;
	}
	
	int32_t intValue = buffer_get_int32_safe(buffer, index, buffer_len);
	float result = (float)intValue / scale;
	
	if (!isfinite(result)) {
		return 0.0f;
	}
	
	return result;
}

float buffer_get_float32_auto_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	uint32_t res = buffer_get_uint32_safe(buffer, index, buffer_len);

	int e = (res >> 23) & 0xFF;
	uint32_t sig_i = res & 0x7FFFFF;
	bool neg = res & (1U << 31);

	float sig = 0.0;
	if (e != 0 || sig_i != 0) {
		sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
		e -= 126;
	}

	if (neg) {
		sig = -sig;
	}

	float result = ldexpf(sig, e);
	
	if (!isfinite(result)) {
		return 0.0f;
	}

	return result;
}

bool buffer_get_bool_safe(const uint8_t *buffer, int32_t *index, int32_t buffer_len) {
	if (!buffer || !index || (*index) < 0 || (*index) >= buffer_len) {
		if (index && (*index) < buffer_len) (*index)++;
		return false;
	}
	
	bool result = (buffer[*index] == 1);
	(*index)++;
	return result;
}
