#include <raylib.h>
#include <raygui.h>

#include "gui.h"
#include "state.h"
#include "world.h"

void draw_main_menu(Game* game, int screen_width, int screen_height) {
    int btn_width = 200;
    int btn_height = 40;
    int center_x = (screen_width - btn_width) / 2;
    int start_y = 300;

    DrawText("JUEGO DE MIERDA", center_x - 80, 150, 40, DARKGRAY);

    if (GuiButton((Rectangle){center_x, start_y, btn_width, btn_height}, "UN PUTO JUGADOR")) {
        game->state = LOBBY;
    }

    if (GuiButton((Rectangle){center_x, start_y + 50, btn_width, btn_height}, "MAS DE UNO")) {
        TraceLog(LOG_INFO, "TODO: Lobby");
    }
}

void draw_config_screen(Game* game, int screen_width, int screen_height) {
    DrawText("CONFIGURACION", 50, 50, 30, DARKGRAY);

    float start_x = 180;
    float start_y = 120;
    float spacing = 60;
    float slider_w = 200;
    float slider_h = 20;

    GuiLabel((Rectangle){start_x, start_y - 25, slider_w, 20}, "JUGADOR");
    GuiSlider((Rectangle){start_x, start_y, slider_w, slider_h}, 
        "Velocidad", TextFormat("%.1f", game->config.player_move_speed), 
        &game->config.player_move_speed, PLAYER_MOVE_SPEED_MIN, PLAYER_MOVE_SPEED_MAX);
    GuiSlider((Rectangle){start_x, start_y + spacing, slider_w, slider_h}, 
        "Salto", TextFormat("%.1f", game->config.player_jump_force), 
        &game->config.player_jump_force, PLAYER_JUMP_FORCE_MIN, PLAYER_JUMP_FORCE_MAX);
    GuiSlider((Rectangle){start_x, start_y + spacing*2, slider_w, slider_h}, 
        "Gravedad", TextFormat("%.1f", game->config.gravity_force), 
        &game->config.gravity_force, GRAVITY_FORCE_MIN, GRAVITY_FORCE_MAX);
    GuiSlider((Rectangle){start_x, start_y + spacing*3, slider_w, slider_h}, 
        "Tamano", TextFormat("%.1f", game->config.player_size), 
        &game->config.player_size, PLAYER_SIZE_MIN, PLAYER_SIZE_MAX);

    float col2_x = start_x + 350;
    GuiLabel((Rectangle){col2_x, start_y - 25, slider_w, 20}, "PODERES");
    GuiSlider((Rectangle){col2_x, start_y, slider_w, slider_h}, 
        "Fuerza Mag", TextFormat("%.0f", game->config.magnet_force_magnitude), 
        &game->config.magnet_force_magnitude, MAGNET_FORCE_MIN, MAGNET_FORCE_MAX);
    GuiSlider((Rectangle){col2_x, start_y + spacing, slider_w, slider_h}, 
        "Radio Mag", TextFormat("%.1f", game->config.magnet_radius), 
        &game->config.magnet_radius, MAGNET_RADIUS_MIN, MAGNET_RADIUS_MAX);
    GuiSlider((Rectangle){col2_x, start_y + spacing*2, slider_w, slider_h}, 
        "Energia", TextFormat("%.1f", game->config.max_energy), 
        &game->config.max_energy, MAX_ENERGY_MIN, MAX_ENERGY_MAX);
    GuiSlider((Rectangle){col2_x, start_y + spacing*3, slider_w, slider_h}, 
        "Recarga", TextFormat("%.1f", game->config.recharge_time), 
        &game->config.recharge_time, RECHARGE_TIME_MIN, RECHARGE_TIME_MAX);

    float col3_x = col2_x + 350;
    GuiLabel((Rectangle){col3_x, start_y - 25, slider_w, 20}, "COMBATE");
    GuiSlider((Rectangle){col3_x, start_y, slider_w, slider_h}, 
        "Dash F", TextFormat("%.1f", game->config.dash_force), 
        &game->config.dash_force, DASH_FORCE_MIN, DASH_FORCE_MAX);
    GuiSlider((Rectangle){col3_x, start_y + spacing, slider_w, slider_h}, 
        "Dash CD", TextFormat("%.1f", game->config.dash_cooldown), 
        &game->config.dash_cooldown, DASH_COOLDOWN_MIN, DASH_COOLDOWN_MAX);
    GuiSlider((Rectangle){col3_x, start_y + spacing*2, slider_w, slider_h}, 
        "Salud", TextFormat("%.0f", game->config.total_health), 
        &game->config.total_health, TOTAL_HEALTH_MIN, TOTAL_HEALTH_MAX);

    float col4_x = col3_x + 350;
    GuiLabel((Rectangle){col4_x, start_y - 25, slider_w, 20}, "ENTORNO / REGLAS");
    GuiSlider((Rectangle){col4_x, start_y, slider_w, slider_h}, 
        "Plat X", TextFormat("%.0f", game->config.main_platform_size.x), 
        &game->config.main_platform_size.x, PLATFORM_X_MIN, PLATFORM_X_MAX);
    GuiSlider((Rectangle){col4_x, start_y + spacing, slider_w, slider_h}, 
        "Plat Y", TextFormat("%.0f", game->config.main_platform_size.y), 
        &game->config.main_platform_size.y, PLATFORM_Y_MIN, PLATFORM_Y_MAX);
    GuiSlider((Rectangle){col4_x, start_y + spacing*2, slider_w, slider_h}, 
        "T. Eventos", TextFormat("%.1f s", game->config.time_between_events), 
        &game->config.time_between_events, TIME_EVENTS_MIN, TIME_EVENTS_MAX);
    float rounds_float = (float)game->config.rounds;

    GuiSlider((Rectangle){col4_x, start_y + spacing*3, slider_w, slider_h}, 
        "Rondas", TextFormat("%d", game->config.rounds), 
        &rounds_float, (float)ROUNDS_MIN, (float)ROUNDS_MAX);
    game->config.rounds = (int)rounds_float;
    
    if (GuiButton((Rectangle){screen_width - 200, screen_height - 100, 150, 50}, "COMENZAR")) {
        init_world(&game->world, 100);
        game->state = PLAYING; 
    }

    if (GuiButton((Rectangle){50, screen_height - 100, 100, 40}, "VOLVER")) {
        game->state = MENU;
    }
}