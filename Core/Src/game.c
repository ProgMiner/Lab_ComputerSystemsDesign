#include "game.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "kb.h"
#include "lcd.h"
#include "sprites.h"


#define GAME_UP_KEY KB_EVENT_KEY_2
#define GAME_PLAY_KEY KB_EVENT_KEY_5
#define GAME_DOWN_KEY KB_EVENT_KEY_8

#define GAME_MAX_SCORE 255

#define GAME_SPACESHIP_WIDTH 22
#define GAME_SPACESHIP_HEIGHT 20
#define GAME_SPACESHIP_X 5
#define GAME_SPACESHIP_MAX_Y (LCD_HEIGHT - GAME_SPACESHIP_HEIGHT)

#define GAME_WALL_WIDTH 4
#define GAME_WALL_HOLE_HEIGHT (2 * GAME_SPACESHIP_HEIGHT)

#define GAME_WALLS_ON_SCREEN 5
#define GAME_WALLS_BUFFER_SIZE (GAME_WALLS_ON_SCREEN + 1)
#define GAME_WALLS_WIDTH_BETWEEN (28 + GAME_WALL_WIDTH)
#define GAME_WALLS_TIME_TO_MOVE 100


enum game_state {

    GAME_STATE_INIT = 0,
    GAME_STATE_IDLE = 1,
    GAME_STATE_GAME = 2,
    GAME_STATE_END = 3,
};

struct hitbox {
    uint8_t x;
    uint8_t y;
    uint8_t width;
    uint8_t height;
};

typedef void (*game_state_function)(enum game_state, uint32_t);


// game state variables

static enum game_state state = GAME_STATE_INIT;
static uint8_t spaceship_y;
static uint8_t walls_hole_ys[GAME_WALLS_BUFFER_SIZE];
static uint32_t walls_time_to_move;
static uint8_t first_wall_idx;
static int8_t first_wall_x;
static uint8_t score;
static bool success;


static bool is_collided(struct hitbox a, struct hitbox b) {
    const uint8_t ax1 = a.x;
    const uint8_t ay1 = a.y;
    const uint8_t ax2 = a.x + a.width;
    const uint8_t ay2 = a.y + a.height;

    const uint8_t bx1 = b.x;
    const uint8_t by1 = b.y;
    const uint8_t bx2 = b.x + b.width;
    const uint8_t by2 = b.y + b.height;

    return ax1 < bx2 && ax2 > bx1 && ay1 < by2 && ay2 > by1;
}

static void game_state_init(enum game_state prev_state, uint32_t dt) {
    (void) prev_state;
    (void) dt;

    state = GAME_STATE_IDLE;
}

static uint8_t generate_wall_hole() {
    return ((uint8_t) rand()) % 25; // NOLINT(cert-msc30-c, cert-msc50-cpp)
}

static void game_start() {
    state = GAME_STATE_GAME;
    spaceship_y = (LCD_HEIGHT - GAME_SPACESHIP_HEIGHT) / 2;

    walls_time_to_move = GAME_WALLS_TIME_TO_MOVE;
    for (uint8_t i = 0; i < GAME_WALLS_ON_SCREEN; ++i) {
        walls_hole_ys[i] = generate_wall_hole();
    }

    first_wall_idx = 0;
    first_wall_x = 127;
    score = 0;
}

static void game_state_idle(enum game_state prev_state, uint32_t dt) {
    (void) dt;

    if (prev_state != GAME_STATE_IDLE) {
        lcd_reset_screen();
        lcd_draw_string(24, 9, &font_16x26, "Lab 4", true, true);
        lcd_draw_string(9, 35, &font_11x18, "Press Play", true, true);
        lcd_done();
        return;
    }

    while (kb_event_has()) {
        struct kb_event evt = kb_event_pop();

        if (evt.type == KB_EVENT_TYPE_RELEASE && evt.key == GAME_PLAY_KEY) {
            game_start();
        }
    }
}

static bool is_spaceship_collided_with_wall() {
    const struct hitbox spaceship = {
        .x = GAME_SPACESHIP_X,
        .y = spaceship_y,
        .width = GAME_SPACESHIP_WIDTH,
        .height = GAME_SPACESHIP_HEIGHT,
    };

    const struct hitbox up_wall = {
        .x = first_wall_x,
        .y = 0,
        .width = GAME_WALL_WIDTH,
        .height = walls_hole_ys[first_wall_idx],
    };

    if (is_collided(spaceship, up_wall)) {
        return true;
    }

    const uint8_t down_wall_y = walls_hole_ys[first_wall_idx] + GAME_WALL_HOLE_HEIGHT;
    const struct hitbox down_wall = {
        .x = first_wall_x,
        .y = down_wall_y,
        .width = GAME_WALL_WIDTH,
        .height = LCD_HEIGHT - down_wall_y,
    };

    if (is_collided(spaceship, down_wall)) {
        return true;
    }

    return false;
}

static void game_state_game(enum game_state prev_state, uint32_t dt) {
    (void) prev_state;

    // DRAW

    lcd_reset_screen();

    uint8_t wall_x = first_wall_x;
    for (uint8_t i = 0, idx = first_wall_idx; i < GAME_WALLS_ON_SCREEN; ++i, idx = (first_wall_idx + 1) % GAME_WALLS_BUFFER_SIZE) {
        lcd_fill_rect(wall_x, 0, wall_x + GAME_WALL_WIDTH - 1, LCD_HEIGHT - 1, true);
        lcd_fill_rect(wall_x, walls_hole_ys[idx], wall_x + GAME_WALL_WIDTH - 1,
                      walls_hole_ys[idx] + GAME_WALL_HOLE_HEIGHT - 1, true);

        wall_x += GAME_WALLS_WIDTH_BETWEEN;

        if (wall_x >= LCD_WIDTH) {
            break;
        }
    }

    lcd_draw_sprite(GAME_SPACESHIP_X, spaceship_y, &spaceship_sprite, true, true);
    lcd_done();

    // UPDATE

    // walls moving
    if (walls_time_to_move <= dt) {
        walls_time_to_move = GAME_WALLS_TIME_TO_MOVE - (dt - walls_time_to_move);

        --first_wall_x;

        // if first wall hide from screen
        if (first_wall_x == -GAME_WALL_WIDTH) {
            first_wall_x += GAME_WALLS_WIDTH_BETWEEN;
            ++first_wall_idx;

            // generate new wall outside of screen
            walls_hole_ys[(first_wall_idx + GAME_WALLS_ON_SCREEN) % GAME_WALLS_BUFFER_SIZE] = generate_wall_hole();
        }
    } else {
        walls_time_to_move -= dt;
    }

    // if spaceship collided with wall
    if (is_spaceship_collided_with_wall()) {
        state = GAME_STATE_END;
        success = false;
        return;
    }

    // if spaceship passed wall
    if (first_wall_x + GAME_WALL_WIDTH < GAME_SPACESHIP_X) {
        ++score;

        // if score reached maximum
        if (score == GAME_MAX_SCORE) {
            state = GAME_STATE_END;
            success = true;
            return;
        }
    }

    // INPUT

    while (kb_event_has()) {
        struct kb_event evt = kb_event_pop();

        if (evt.type != KB_EVENT_TYPE_PRESS) {
            continue;
        }

        switch (evt.key) {
            case GAME_UP_KEY:
                if (spaceship_y > 0) {
                    --spaceship_y;
                }

                break;

            case GAME_DOWN_KEY:
                if (spaceship_y < GAME_SPACESHIP_MAX_Y) {
                    ++spaceship_y;
                }

                break;

            case GAME_PLAY_KEY:
                state = GAME_STATE_END;
                success = true;
                break;

            default:
                // do nothing
                break;
        }
    }
}

static void game_print_score() {
    char score_str[13];

    const int len = snprintf(score_str, 13, "Score: %u", score);
    score_str[12] = '\0';

    lcd_draw_string((LCD_WIDTH - len * 11) / 2, 35, &font_11x18, score_str, true, true);
}

static void game_state_end(enum game_state prev_state, uint32_t dt) {
    (void) dt;

    if (prev_state != GAME_STATE_END) {
        lcd_reset_screen();

        if (success) {
            lcd_draw_string(8, 9, &font_16x26, "Success", true, true);
        } else {
            lcd_draw_string(32, 9, &font_16x26, "Fail", true, true);
        }

        game_print_score();
        lcd_done();
        return;
    }

    while (kb_event_has()) {
        struct kb_event evt = kb_event_pop();

        if (evt.type == KB_EVENT_TYPE_RELEASE && evt.key == GAME_PLAY_KEY) {
            state = GAME_STATE_IDLE;
        }
    }
}

void game_step() {
    static const game_state_function game_state_functions[] = {
        [GAME_STATE_INIT] = game_state_init,
        [GAME_STATE_IDLE] = game_state_idle,
        [GAME_STATE_GAME] = game_state_game,
        [GAME_STATE_END] = game_state_end,
    };

    static enum game_state prev_state = GAME_STATE_INIT;
    static uint32_t prev_t = 0;

    const enum game_state saved_state = state;
    const uint32_t t = HAL_GetTick();

    game_state_functions[state](prev_state, t - prev_t);

    prev_state = saved_state;
    prev_t = t;
}
