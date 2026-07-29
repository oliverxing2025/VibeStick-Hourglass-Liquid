#include "hourglass_chime.h"

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vibe_board.h"

#define PIN_ES8311_MCLK 18
#define PIN_ES8311_BCLK 17
#define PIN_ES8311_LRCK 15
#define PIN_ES8311_DIN 14

#define CHIME_SAMPLE_RATE 16000
#define CHIME_FRAME_SAMPLES 160
#define CHIME_FADE_MS 12
#define CHIME_TWO_PI 6.28318530717958647692f

typedef struct {
    uint16_t frequency_hz;
    uint16_t duration_ms;
} chime_note_t;

static const char *TAG = "hourglass_chime";
static atomic_bool s_playing;
static i2s_chan_handle_t s_tx_handle;
static esp_codec_dev_handle_t s_codec;
static const audio_codec_ctrl_if_t *s_ctrl_if;
static const audio_codec_data_if_t *s_data_if;
static const audio_codec_gpio_if_t *s_gpio_if;
static const audio_codec_if_t *s_codec_if;

static void release_audio(void)
{
    if (s_codec) {
        esp_codec_dev_close(s_codec);
        esp_codec_dev_delete(s_codec);
        s_codec = NULL;
    }
    if (s_codec_if) {
        audio_codec_delete_codec_if(s_codec_if);
        s_codec_if = NULL;
    }
    if (s_data_if) {
        audio_codec_delete_data_if(s_data_if);
        s_data_if = NULL;
    }
    if (s_gpio_if) {
        audio_codec_delete_gpio_if(s_gpio_if);
        s_gpio_if = NULL;
    }
    if (s_ctrl_if) {
        audio_codec_delete_ctrl_if(s_ctrl_if);
        s_ctrl_if = NULL;
    }
    if (s_tx_handle) {
        /* esp_codec_dev_close() already stops the shared TX channel. */
        i2s_del_channel(s_tx_handle);
        s_tx_handle = NULL;
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(vibe_board_speaker_set_enabled(false));
}

static esp_err_t open_audio(void)
{
    ESP_RETURN_ON_ERROR(
        vibe_board_speaker_set_enabled(true), TAG, "speaker amp");

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    channel_config.auto_clear = true;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&channel_config, &s_tx_handle, NULL),
        TAG, "create I2S");

    i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CHIME_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = PIN_ES8311_MCLK,
            .bclk = PIN_ES8311_BCLK,
            .ws = PIN_ES8311_LRCK,
            .dout = PIN_ES8311_DIN,
            .din = I2S_GPIO_UNUSED,
        },
    };
    i2s_config.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_tx_handle, &i2s_config),
        TAG, "configure I2S");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_tx_handle), TAG, "enable I2S");

    audio_codec_i2c_cfg_t i2c_config = {
        .port = I2C_NUM_1,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = vibe_board_i2c_bus(),
    };
    s_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_config);
    ESP_RETURN_ON_FALSE(s_ctrl_if, ESP_ERR_NO_MEM, TAG, "codec control");

    audio_codec_i2s_cfg_t data_config = {
        .port = I2S_NUM_1,
        .tx_handle = s_tx_handle,
    };
    s_data_if = audio_codec_new_i2s_data(&data_config);
    s_gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(
        s_data_if && s_gpio_if, ESP_ERR_NO_MEM, TAG, "codec interfaces");

    es8311_codec_cfg_t codec_config = {
        .ctrl_if = s_ctrl_if,
        .gpio_if = s_gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = -1,
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
    };
    s_codec_if = es8311_codec_new(&codec_config);
    ESP_RETURN_ON_FALSE(s_codec_if, ESP_ERR_NO_MEM, TAG, "ES8311");

    esp_codec_dev_cfg_t device_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = s_codec_if,
        .data_if = s_data_if,
    };
    s_codec = esp_codec_dev_new(&device_config);
    ESP_RETURN_ON_FALSE(s_codec, ESP_ERR_NO_MEM, TAG, "codec device");

    esp_codec_dev_sample_info_t sample_config = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = 1,
        .channel_mask = I2S_STD_SLOT_LEFT,
        .sample_rate = CHIME_SAMPLE_RATE,
    };
    ESP_RETURN_ON_FALSE(
        esp_codec_dev_open(s_codec, &sample_config) == ESP_CODEC_DEV_OK,
        ESP_FAIL, TAG, "open codec");
    ESP_RETURN_ON_FALSE(
        esp_codec_dev_set_out_vol(s_codec, 78) == ESP_CODEC_DEV_OK,
        ESP_FAIL, TAG, "set volume");
    ESP_RETURN_ON_FALSE(
        esp_codec_dev_set_out_mute(s_codec, false) == ESP_CODEC_DEV_OK,
        ESP_FAIL, TAG, "unmute");
    return ESP_OK;
}

static esp_err_t play_note(const chime_note_t *note)
{
    int total_samples =
        CHIME_SAMPLE_RATE * (int)note->duration_ms / 1000;
    int fade_samples = CHIME_SAMPLE_RATE * CHIME_FADE_MS / 1000;
    int written = 0;
    int16_t frame[CHIME_FRAME_SAMPLES];
    while (written < total_samples) {
        int frame_samples = total_samples - written;
        if (frame_samples > CHIME_FRAME_SAMPLES) {
            frame_samples = CHIME_FRAME_SAMPLES;
        }
        for (int i = 0; i < frame_samples; ++i) {
            int sample = written + i;
            if (note->frequency_hz == 0) {
                frame[i] = 0;
                continue;
            }
            float envelope = 1.0f;
            if (sample < fade_samples) {
                envelope = (float)sample / (float)fade_samples;
            } else if (total_samples - sample - 1 < fade_samples) {
                envelope =
                    (float)(total_samples - sample - 1) /
                    (float)fade_samples;
            }
            float phase =
                CHIME_TWO_PI * (float)note->frequency_hz *
                (float)sample / (float)CHIME_SAMPLE_RATE;
            float wave = sinf(phase) + 0.12f * sinf(phase * 2.0f);
            frame[i] = (int16_t)(
                wave * envelope * 0.13f * 32767.0f);
        }
        int bytes = frame_samples * (int)sizeof(frame[0]);
        ESP_RETURN_ON_FALSE(
            esp_codec_dev_write(s_codec, frame, bytes) == ESP_CODEC_DEV_OK,
            ESP_FAIL, TAG, "write note");
        written += frame_samples;
    }
    return ESP_OK;
}

static void chime_task(void *argument)
{
    (void)argument;
    static const chime_note_t melody[] = {
        {659, 115},
        {0, 28},
        {784, 135},
        {0, 28},
        {1047, 240},
        {0, 35},
    };
    esp_err_t err = open_audio();
    if (err == ESP_OK) {
        for (size_t i = 0; i < sizeof(melody) / sizeof(melody[0]); ++i) {
            err = play_note(&melody[i]);
            if (err != ESP_OK) {
                break;
            }
        }
    }
    release_audio();
    atomic_store(&s_playing, false);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "completion chime played");
    } else {
        ESP_LOGW(TAG, "completion chime failed: %s", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

esp_err_t hourglass_chime_play(void)
{
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_playing, &expected, true)) {
        return ESP_ERR_INVALID_STATE;
    }
    BaseType_t created = xTaskCreate(
        chime_task, "hourglass_chime", 6144, NULL, 4, NULL);
    if (created != pdPASS) {
        atomic_store(&s_playing, false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
