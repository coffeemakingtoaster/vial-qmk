/* Copyright 2023 splitkb.com <support@splitkb.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H

#include "rev1.h"

#include "spi_master.h"
#include "matrix.h"
#include "keyboard.h"

#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include "myriad.h"

// Needed for early boot
#include "hardware/xosc.h"


bool is_oled_enabled = true;

int spaceship_dodge_direction = 0;

int coffee_frame_counter = 0;

typedef struct Entity {
    int x;
    int y;
    // This assumes the entities to have a quadrat like shape
    int size;
} Entity;

int current_obstacle_count = 0;

int highest_obstacle_y = 0;

// Result of (128 * 64)/ 8 -> 1 bit for every pixel
char PROGMEM frame_buffer [1024];

Entity obstacle_array[3];

Entity spaceship = {
    32,
    96,
    5
};

//// Early boot

// The Elora has support for Myriad Controller Modules.
// Essentially, this is a plug-in controller which takes over all functionality from the onboard MCU.
// This makes it possible to convert the keyboard into a wireless board, for example.
// In order for this to work, we need to prevent QMK from booting.
static void enter_standby_mode(void) {
    while (true) {
        // Todo: Look into more approaches to save power
        // - deinit PLL
        // - MEMPOWERDOWN
        // - QSPI power-down (idle use is 10-50 uA)

        // The RP2040 *should* be able to power-down to about 180uA,
        // while the QSPI chip can do 1-15 uA.

        // Additional 3V3 power usage which can't be disabled:
        // - Matrix SPI NOT gate: 0.1-4 uA
        // - Matrix SPI tri-state buffer: 0.1-10 uA
        // - Shift registers: 5x 0.1-2 uA

        // Turns off the crystal oscillator until the core is woken by an interrupt.
        // This will block and hence the entire system will stop, until an interrupt wakes it up.
        // This function will continue to block until the oscillator becomes stable after its wakeup.
        xosc_dormant();
    }
}

void early_hardware_init_pre(void) {
    // GP2 has an external pullup. It is shorted to ground when a controller module is connected.
    setPinInput(GP2);
    if (!readPin(GP2)) {
        // A Myriad Controller Module is present,
        // so we are not needed to run QMK.
        enter_standby_mode();
    }
}

//// HW init

// Make sure all external hardware is
// in a known-good state on powerup
void keyboard_pre_init_kb(void) {
    /// SPI Chip Select pins for various hardware
    // Matrix CS
    setPinOutput(GP13);
    writePinHigh(GP13);
    // Myriad Module CS
    setPinOutput(GP9);
    writePinHigh(GP9);

    setPinOutput(ELORA_CC1_PIN);
    writePinLow(ELORA_CC1_PIN);

    setPinOutput(ELORA_CC2_PIN);
    writePinLow(ELORA_CC2_PIN);

    // We have to get the SPI interface working quite early,
    // So make sure it is available well before we need it
    spi_init();

    keyboard_pre_init_user();
}

void keyboard_post_init_kb(void) {

    keyboard_post_init_user();
}

//// Matrix functionality

// The matrix is hooked up to a chain of 74xx165 shift registers.
// Pin F0 acts as Chip Select (active-low)
// The signal goes to a NOT gate, whose output is wired to
// a) the latch pin of the shift registers
// b) the "enable" pin of a tri-state buffer,
//    attached between the shift registers and MISO
// F0 has an external pull-up.
// SCK works as usual.
//
// Note that the matrix contains a variety of data.
// In addition to the keys, it also reads the rotary encoders
// and whether the board is the left/right half.

void matrix_init_custom(void) {
    // Note: `spi_init` has already been called
    // in `keyboard_pre_init_kb()`, so nothing to do here
}

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    // Enough to hold the shift registers
    uint16_t length = 5;
    uint8_t data[length];
    spi_status_t res;

    // Matrix SPI config
    // 1) Pin
    // 2) Mode: Register shifts on rising clock, and clock idles low
    //      pol = 0 & pha = 0 => mode 0
    // 3) LSB first: Register outputs H first, and we want H as MSB,
    //      as this result in a neat A-H order in the layout macro.
    // 4) Divisor: 2 is the fastest possible, at Fclk / 2.
    //      range is 2-128
    spi_start(GP13, false, 0, 128);
    res = spi_receive(data, length);
    spi_stop();
    if (res != SPI_STATUS_SUCCESS) {
        dprint("ERROR: SPI timed out while reading matrix!");
    }

    bool matrix_has_changed = false;
    for (uint8_t i = 0; i < length; i++) {
        // Bitwise NOT because we use pull-ups,
        // and switches short the pin to ground,
        // but the matrix uses 1 to indicate a pressed switch
        uint8_t word = ~data[i];
        matrix_has_changed |= current_matrix[i] ^ word;
        current_matrix[i] = word;
    }
    #ifdef MYRIAD_ENABLE
    // It's a bit of a weird place to call a `_task`,
    // but we want to do it relatively early because we mess with a lot of functionality
    myriad_task();
    return matrix_has_changed || myriad_hook_matrix(current_matrix);
    #else
    return matrix_has_changed;
    #endif
}

//// Encoder functionality

// The encoders are hooked in to the same shift registers as the switch matrix, so we can just piggyback on that.

// Clone of a variant in quantum/matrix_common.c, but matrix-agnostic
bool mat_is_on(matrix_row_t mat[], uint8_t row, uint8_t col) {
    return (mat[row] & ((matrix_row_t)1 << col));
}

void encoder_read_pads_from(bool pads[], matrix_row_t mat[]) {
    // First two matrix rows:
    //
    // Pin  A   B   C   D   E   F   G   H
    // Left:
    //   { __, __, __, __, __, __, A1, B1 },
    //   { A3, B3, A2, B2, __, __, __, __ }
    // Right:
    //   { A1, B1, __, __, __, __, __, __ },
    //   { __, __, __, __, A2, B2, A3, B3 }
    //
    // See also rev1.h

    if (is_keyboard_left()) {
        pads[0] = mat_is_on(mat, 0, 6);
        pads[1] = mat_is_on(mat, 0, 7);
        pads[2] = mat_is_on(mat, 1, 2);
        pads[3] = mat_is_on(mat, 1, 3);
        pads[4] = mat_is_on(mat, 1, 0);
        pads[5] = mat_is_on(mat, 1, 1);
    } else {
        pads[0] = mat_is_on(mat, 0, 0);
        pads[1] = mat_is_on(mat, 0, 1);
        pads[2] = mat_is_on(mat, 1, 4);
        pads[3] = mat_is_on(mat, 1, 5);
        pads[4] = mat_is_on(mat, 1, 6);
        pads[5] = mat_is_on(mat, 1, 7);
    }
}

void encoder_init_pads(uint8_t count, bool pads[]) {
    // At this point the first matrix scan hasn't happened yet,
    // so we can't use raw_matrix to initialize our encoder state
    // as it contains all zeroes - so we have to do our own first scan
    matrix_row_t mat[MATRIX_ROWS];
    matrix_scan_custom(mat);

    encoder_read_pads_from(pads, mat);

    // Make sure the Myriad pads are in a well-defined state
    pads[6] = 0;
    pads[7] = 0;
    #ifdef MYRIAD_ENABLE
    myriad_hook_encoder(count, pads);
    #endif
}

extern matrix_row_t raw_matrix[MATRIX_ROWS]; // From quantum/matrix_common.c
void encoder_read_pads(uint8_t count, bool pads[]) {
    // The matrix code already keeps the raw matrix up-to-date,
    // so we only have to read the values from it
    encoder_read_pads_from(pads, raw_matrix);

    // Make sure the Myriad pads are in a well-defined state
    pads[6] = 0;
    pads[7] = 0;
    #ifdef MYRIAD_ENABLE
    myriad_hook_encoder(count, pads);
    #endif
}

//// Default functionality

// RGB Matrix definition for Elora
#ifdef RGB_MATRIX_ENABLE

#define NLD NO_LED

// Layout
//     2                          1                            0                  6                            7                          8
//     ┌───────────────────────────────────────────┐                                          ┌───────────────────────────────────────────┐
//     │ MX101, MX105, MX109, MX113, MX117, MX121, │                                          │ MX221, MX217, MX213, MX209, MX205, MX201, │
//     ├───────────────────────────────────────────┤                                          ├───────────────────────────────────────────┤
//     │ MX102, MX106, MX110, MX114, MX118, MX122, │                                          │ MX222, MX218, MX214, MX210, MX206, MX202, │
//     ├───────────────────────────────────────────┤                                          ├───────────────────────────────────────────┤
//     │ MX103, MX107, MX111, MX115, MX119, MX123, │                                          │ MX223, MX219, MX215, MX211, MX207, MX203, │
//     ├───────────────────────────────────────────┴─────────────┐              ┌─────────────┴───────────────────────────────────────────┤
//     │ MX104, MX108, MX112, MX116, MX120, MX124, MX131, MX130, │              │ MX230, MX231, MX224, MX220, MX216, MX212, MX208, MX204, │
//     └────────────────────┬────────────────────────────────────┤              ├───────────────────────────────────┬─────────────────────┘
//     3                    │ MX125, MX126, MX127, MX128, MX129, │              │ MX229, MX228, MX227, MX226, MX225 │                     9
//                          └────────────────────────────────────┘              └───────────────────────────────────┘
//                                4                            5                 11                           10

// #define LAYOUT_myr(
//     2 2 2 1 1 0        NLD NLD         6  7  7 8 8 8
//     2 2 2 1 1 0        NLD NLD         6  7  7 8 8 8
//     3 3 3 4 4 0        NLD NLD         6 10 10 9 9 9
//     3 3 3 4 4 4 5 5             11 11 10 10 10 9 9 9
//             3 4 4 5 5        11 11 10 10  9
//     NLD NLD NLD NLD NLD          NLD NLD NLD NLD NLD
// )


led_config_t g_led_config = {
    {
    //COL  01   02   03   04   05   011   010   09    ROW
        {   5, NLD, NLD, NLD,   5,   5, NLD, NLD }, // 00
        { NLD, NLD, NLD, NLD,   5,   4,   4,   3 }, // 01
        {   1,   1,   0,   0,   0,   4,   4,   4 }, // 02
        {   2,   2,   1,   1,   4,   4,   3,   3 }, // 03
        {   2,   2,   2,   2,   3,   3,   3,   3 }, // 04
        { NLD, NLD, NLD, NLD, NLD, NLD, NLD, NLD }, // 05

        { NLD, NLD,  11,  11, NLD, NLD, NLD,  11 }, // 06
        {   9,  10,  10,  11, NLD, NLD, NLD, NLD }, // 07
        {   10, 10,  10,   6,   6,   6,   7,   7,}, // 08
        {   9,   9,  10,  10,   7,   7,   8,   8 }, // 09
        {   9,   9,   9,   9,   8,   8,   8,   8 }, // 10
        { NLD, NLD, NLD, NLD, NLD, NLD, NLD, NLD }, // 11
    },
    {
        // { 112, 32 } is the center
        {90 , 0},  // 0
        {45 , 0},  // 1
        {0  , 0},  // 2
        {0  , 64}, // 3
        {45 , 64}, // 4
        {90 , 64}, // 5
        {134, 0},  // 6
        {179, 0},  // 7
        {224, 0},  // 8
        {224, 64}, // 9
        {179, 64}, // 10
        {134, 64}   // 11
    },
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    }
};
#endif

#ifdef OLED_ENABLE
oled_rotation_t oled_init_kb(oled_rotation_t rotation) {
    if (is_keyboard_left()) {
        return OLED_ROTATION_270;
    } else {
        return OLED_ROTATION_90;
    }
    for (int i = 0; i< 1024; i++){
        frame_buffer[i] = 0x00;
    }
}

void spawn_obstacle(int index){
        obstacle_array[index].x = rand() % ((63 - 5) + 1 - 0) + 0;
        obstacle_array[index].y = 10;
        obstacle_array[index].size = 10;
}

void draw_player(int x, int y, int size){
    for (int c_x=x; c_x<(x+size); c_x++){
        oled_write_pixel(c_x, y, true);
        oled_write_pixel(c_x, y+size, true);
    }
    for (int c_y = y; c_y<y+size; c_y++){
        oled_write_pixel(x, c_y, true);
        oled_write_pixel(x + size, c_y, true);
    }
}

void draw_rect(int x, int y, int size, bool on){
    for (int c_x=x; c_x<(x+size); c_x++){
        for (int c_y=y; c_y<(y+size); c_y++){
            oled_write_pixel(c_x, c_y, on);
        }
    }
}

bool oled_task_kb(void) {
    if (!oled_task_user()) {
        return false;
    }

    if (!is_oled_enabled) {
        oled_off();
    } else  {
        oled_on();
    }

    if (is_keyboard_master()) {
        // Check if new obstacle can and should be spawned
        if (current_obstacle_count == 0){
            spawn_obstacle(0);
            current_obstacle_count++;
        } else if (current_obstacle_count < 3 && highest_obstacle_y >= 40){
            spawn_obstacle(current_obstacle_count);
            current_obstacle_count++;
        }

        oled_set_cursor(0, 0);
        oled_write_ln_P(PSTR(" ! DODGE ! "), false);

        int minimum = 100;
        int danger_x = 255;

        // update obstacles
        for (int i = 0; i < current_obstacle_count; i++){

            obstacle_array[i].y = obstacle_array[i].y + 1;

            if (obstacle_array[i].y < minimum){
                minimum = obstacle_array[i].y;
            }

            if (obstacle_array[i].y > 60 && obstacle_array[i].y < 90){
                danger_x = obstacle_array[i].x;
            }

            // clear old rect
            draw_rect(obstacle_array[i].x, obstacle_array[i].y - 1, obstacle_array[i].size, false);
            draw_rect(obstacle_array[i].x, obstacle_array[i].y, obstacle_array[i].size, true);

            // did obstacle leave frame?
            if (obstacle_array[i].y > 128){
                srand(obstacle_array[i].x);
                draw_rect(obstacle_array[i].x, obstacle_array[i].y, obstacle_array[i].size, false);
                obstacle_array[i].y = 10;
                obstacle_array[i].x = rand() % ((63 - 10) + 1 - 0) + 0;
            }
        }

        highest_obstacle_y = minimum;

        // Player logic
        draw_rect(spaceship.x, spaceship.y, spaceship.size + 1, false);

        // Does spaceship have to dodge?
        if (((spaceship.x - danger_x < 30  && spaceship.x - danger_x >= 0 ) ||
            (spaceship.x - danger_x > -30 && spaceship.x - danger_x <= 0 )) && danger_x != 255){
            if (danger_x >= 32){
                spaceship.x--;
            } else {
                 spaceship.x++;
            }
        }

        draw_player(spaceship.x, spaceship.y, spaceship.size);


    } else {
        // Elora sigil

            oled_set_cursor(0, 2);

            if (coffee_frame_counter < 5){
                oled_write_raw_P(coffee_frame_1, sizeof(coffee_frame_1));
            } else if (coffee_frame_counter >= 5 && coffee_frame_counter < 10){
                oled_write_raw_P(coffee_frame_2, sizeof(coffee_frame_2));
            } else if (coffee_frame_counter >= 10 && coffee_frame_counter < 15){
                oled_write_raw_P(coffee_frame_3, sizeof(coffee_frame_3));
            } else {
                oled_write_raw_P(coffee_frame_4, sizeof(coffee_frame_4));
            }

            coffee_frame_counter = (coffee_frame_counter + 1) % 20;

            oled_set_cursor(0, oled_max_lines()-5);
            oled_write_ln_P(PSTR("Relax...\nit`s\ncoffee\ntime"), false);

        }

        return false;
    }

    void housekeeping_task_kb(void) {
        is_oled_enabled = last_input_activity_elapsed() < 60000;
    }
#endif

#ifdef ENCODER_ENABLE
    bool encoder_update_kb(uint8_t index, bool clockwise) {
        if (!encoder_update_user(index, clockwise)) {
            return false;
        }

    if (index == 0 || index == 1 || index == 2) {
        // Left side
        // Arrow keys
        if (clockwise) {
            tap_code(KC_RIGHT);
        } else {
            tap_code(KC_LEFT);
        }
    } else if (index == 4 || index == 5 || index == 6) {
        // Right side
        // Page up/Page down
        if (clockwise) {
            tap_code(KC_PGDN);
        } else {
            tap_code(KC_PGUP);
        }
    } else if (index == 3 || index == 7) {
        // Myriad
        // Volume control
        if (clockwise) {
            tap_code(KC_VOLU);
        } else {
            tap_code(KC_VOLD);
        }
    }
    return true;
}
#endif
