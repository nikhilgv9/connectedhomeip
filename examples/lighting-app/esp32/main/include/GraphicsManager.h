/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include <stdint.h>
#include <cstring>
#include "esp_err.h"
#include "esp_heap_caps.h"

#define IMAGE_W 320
#define IMAGE_H 240

/**
 * @brief Calculate the effect for a bunch of lines.
 *
 * @param dest Destination for the pixels. Assumed to be LINECT * 320 16-bit pixel values.
 * @param line Starting line of the chunk of lines.
 * @param linect Amount of lines to calculate
 */
void pretty_effect_calc_lines(uint16_t *dest, int line, int linect);


void init_blank_screen();

esp_err_t draw_smiley_face(void);

esp_err_t draw_sad_face(void);

esp_err_t draw_close_eye(uint8_t eye);

esp_err_t draw_open_eye(uint8_t eye);

esp_err_t draw_neutral_lips();

esp_err_t draw_smile_lips();

