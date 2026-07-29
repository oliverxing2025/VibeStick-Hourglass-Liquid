idf_component_register(
    SRCS "main.c" "hourglass_physics.c" "hourglass_chime.c" "vibe_board.c"
         "../third_party/bmi270/bmi2.c" "../third_party/bmi270/bmi270.c"
    INCLUDE_DIRS "../include" "../third_party/bmi270"
    REQUIRES app_update driver esp_timer esp_lcd espressif__esp_codec_dev
             espressif__button lvgl__lvgl
)
