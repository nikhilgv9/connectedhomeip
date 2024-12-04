/*
   This code generates an effect that should pass the 'fancy graphics' qualification
   as set in the comment in the spi_master code.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <math.h>
#include "GraphicsManager.h"

uint8_t *pixels;

static inline uint16_t get_bgnd_pixel(int x, int y)
{
    int x0 = x / 8;
    int x1 = x % 8;
    int val = *(pixels + (y * IMAGE_W / 8) + x0) & (1 << x1);
    if (val != 0) {
      return (uint16_t)156;
    } else {
      return (uint16_t)0;
    }
    return (uint16_t) *(pixels + (y * IMAGE_W) + x);
}

static inline void set_pixel(int x, int y, bool color) {
    int x0 = x / 8;
    int x1 = x % 8;
    if (color) { // Set pixel
      *(pixels + (y * IMAGE_W / 8) + x0) |= (1 << x1);
    } else { // Erase Pixel
      *(pixels + (y * IMAGE_W / 8) + x0) &= ~(1 << x1);
    }

  // *(pixels + (y * IMAGE_W) + x) = color;
}

//Calculate the pixel data for a set of lines (with implied line size of 320). Pixels go in dest, line is the Y-coordinate of the
//first line to be calculated, linect is the amount of lines to calculate. Frame increases by one every time the entire image
//is displayed; this is used to go to the next frame of animation.
void pretty_effect_calc_lines(uint16_t *dest, int line, int linect)
{
    for (int y=line; y<line+linect; y++) {
      for (int x=0; x<IMAGE_H; x++) {
          *dest++= get_bgnd_pixel(y, x);
      }
    }
}

void draw_circle(int x, int y, int r, bool color = true) {
  for (int i = x - r; i <= x + r; i++) {
    int j1 = y + sqrt((double)r*r - (i - x)*(i-x));
    int j2 = y - sqrt((double)r*r - (i - x)*(i-x));
    set_pixel(i, j1, color);
    set_pixel(i, j2, color);
  }

  for (int j = y - r; j <= y + r; j++) {
    int i1 = x + sqrt((double)r*r - (j - y)*(j-y));
    int i2 = x - sqrt((double)r*r - (j - y)*(j-y));
    set_pixel(i1, j, color);
    set_pixel(i2, j, color);
  }
}

void draw_semi_circle(int x, int y, int r, bool color = true) {
  for (int i = x - r; i <= x + r; i++) {
    int j1 = y + sqrt((double)r*r - (i - x)*(i-x));
    // int j2 = y - sqrt((double)r*r - (i - x)*(i-x));
    set_pixel(i, j1, color);
//    set_pixel(i, j2, (uint16_t) 156);
  }

  for (int j = y; j <= y + r; j++) {
    int i1 = x + sqrt((double)r*r - (j - y)*(j-y));
    int i2 = x - sqrt((double)r*r - (j - y)*(j-y));
    set_pixel(i1, j, color);
    set_pixel(i2, j, color);
  }
}

void draw_semi_circle_inv(int x, int y, int r, bool color = true) {
  for (int i = x - r; i <= x + r; i++) {
//    int j1 = y + sqrt((double)r*r - (i - x)*(i-x));
    int j2 = y - sqrt((double)r*r - (i - x)*(i-x));
//    set_pixel(i, j1, (uint16_t) 156);
    set_pixel(i, j2, color);
  }

  for (int j = y - r; j <= y; j++) {
    int i1 = x + sqrt((double)r*r - (j - y)*(j-y));
    int i2 = x - sqrt((double)r*r - (j - y)*(j-y));
    set_pixel(i1, j, color);
    set_pixel(i2, j, color);
  }
}

void draw_line(int x1, int y1, int x2, int y2, bool color = true) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  if (dx == 0) {
    // Perpendicular line
    for (int j = y1; j <= y2; j++) {
      set_pixel(x1, j, color);
    }
  } else {
    for (int i = x1; i <= x2; i++) {
      int j = y1 + dy * (i-x1)/dx;
      set_pixel(i, j, color);
    }
  }
}

void init_blank_screen() {
  pixels = (uint8_t*)malloc(IMAGE_H * IMAGE_W / 8);
}

esp_err_t draw_smiley_face(void)
{
    memset(pixels, 0, IMAGE_H * IMAGE_W / 8);
    draw_circle(160, 120, 100);
    draw_circle(120, 80, 20);
    draw_circle(200, 80, 20);
    draw_semi_circle(160, 130, 50);
    return ESP_OK;
}

esp_err_t draw_sad_face(void)
{
    memset(pixels, 0, IMAGE_H * IMAGE_W / 8);
    draw_circle(160, 120, 100);
    draw_circle(120, 80, 20);
    draw_circle(200, 80, 20);
    draw_semi_circle_inv(160, 180, 50);
    return ESP_OK;
}

esp_err_t draw_neutral_lips()
{
  // Erase possible other lips
  draw_semi_circle(160, 130, 50, false);
  draw_semi_circle_inv(160, 180, 50, false);
  draw_line(110, 150, 210, 150);
  return ESP_OK;
}

esp_err_t draw_smile_lips()
{
  // Erase possible other lips
  draw_semi_circle_inv(160, 180, 50, false);
  draw_line(110, 150, 210, 150, false);
  draw_semi_circle(160, 130, 50);
  return ESP_OK;
}

esp_err_t draw_close_eye(uint8_t eye)
{
    // memset(pixels, 0, IMAGE_H * IMAGE_W / 8);
    if (eye == 0) {
      // Left
      draw_circle(120, 80, 20, false); // Erase Left
      draw_semi_circle(120, 80, 20); // Draw closed eye
    } else if (eye == 1) {
      // Right
      draw_circle(200, 80, 20, false); // Erase Right
      draw_semi_circle(200, 80, 20); // Draw closed eye
    } else {
      // Both
      draw_circle(120, 80, 20, false); // Erase Left
      draw_circle(200, 80, 20, false); // Erase Right
      draw_semi_circle(120, 80, 20); // Draw closed eye
      draw_semi_circle(200, 80, 20); // Draw closed eye
    }

    return ESP_OK;
}

esp_err_t draw_open_eye(uint8_t eye)
{
    // memset(pixels, 0, IMAGE_H * IMAGE_W / 8);
    if (eye == 0) {
      // Left
      draw_circle(120, 80, 20); // Draw Left
//      draw_semi_circle(120, 80, 20); // Draw closed eye
    } else if (eye == 1) {
      // Right
      draw_circle(200, 80, 20); // Draw Right
//      draw_semi_circle(200, 80, 20); // Draw closed eye
    } else {
      // Both
      draw_circle(120, 80, 20); // Draw Left
      draw_circle(200, 80, 20); // Draw Right
//      draw_semi_circle(120, 80, 20); // Draw closed eye
//      draw_semi_circle(200, 80, 20); // Draw closed eye
    }

    return ESP_OK;
}

