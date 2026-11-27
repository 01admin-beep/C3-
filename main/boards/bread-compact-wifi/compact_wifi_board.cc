#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "stepper_motor_controller.h"
#include <driver/gpio.h>
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    StepperMotorController* stepper_motor_;  /**< 步进电机控制器 */
    uint8_t display_addr_ = 0x3C;            // 探测到的 OLED I2C 地址
    bool has_oled_ = false;                  // OLED 初始化成功后置位，防止误强转 NoDisplay

    void InitializeDisplayI2c() {
        // ── 自动探测 OLED 的 I2C 引脚与地址 ──
        // 不同小智开发板的 OLED 引脚定义不同（41/42、4/15、7/8、15/16、12/13 等），
        // 逐组创建 I2C 总线并 probe 常见地址 0x3C/0x3D，命中即用，避免刷错板型黑屏。
        static const int kPinCandidates[][2] = {
            {DISPLAY_SDA_PIN, DISPLAY_SCL_PIN},  // config.h 默认配置优先
            {41, 42}, {4, 15}, {7, 8}, {15, 16}, {12, 13},
            {13, 12}, {16, 15}, {8, 9}, {6, 7},
        };
        static const uint8_t kAddrs[] = {0x3C, 0x3D};

        for (const auto& pins : kPinCandidates) {
            int sda = pins[0], scl = pins[1];
            if (sda < 0 || scl < 0) continue;      // GPIO_NUM_NC 跳过
            if (sda == scl) continue;
            i2c_master_bus_config_t bus_config = {
                .i2c_port = (i2c_port_t)0,
                .sda_io_num = (gpio_num_t)sda,
                .scl_io_num = (gpio_num_t)scl,
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7,
                .intr_priority = 0,
                .trans_queue_depth = 0,
                .flags = {
                    .enable_internal_pullup = 1,
                },
            };
            i2c_master_bus_handle_t bus = nullptr;
            if (i2c_new_master_bus(&bus_config, &bus) != ESP_OK) continue;

            bool found = false;
            for (uint8_t addr : kAddrs) {
                if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
                    display_i2c_bus_ = bus;
                    display_addr_ = addr;
                    found = true;
                    ESP_LOGI(TAG, "OLED found: SDA=%d SCL=%d addr=0x%02X", sda, scl, addr);
                    break;
                }
            }
            if (found) return;
            i2c_del_master_bus(bus);
        }
        ESP_LOGE(TAG, "OLED not found on any known I2C pins, display disabled");
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = display_addr_,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        has_oled_ = true;
        // 恢复用户上次选择的待机壁纸（网页 /api/theme 切换后保存在 NVS）
        static_cast<OledDisplay*>(display_)->SetWallpaper(
            Settings("display").GetInt("wallpaper", 0));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        boot_button_.OnLongPress([this]() {
            // 长按 BOOT 循环切换待机壁纸（与网页 /api/theme 一致，切完存 NVS）
            auto* oled = static_cast<OledDisplay*>(GetDisplay());
            if (!has_oled_ || !oled) return;
            int wp = oled->NextWallpaper();
            Settings("display", true).SetInt("wallpaper", wp);
            ESP_LOGI(TAG, "wallpaper -> %d (%s)", wp, OledDisplay::WallpaperName(wp));
        });
        touch_button_.OnPressDown([]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
        // 初始化步进电机控制器
        stepper_motor_ = new StepperMotorController((gpio_num_t)STEPPER_IN1_GPIO, (gpio_num_t)STEPPER_IN2_GPIO,
                                                   (gpio_num_t)STEPPER_IN3_GPIO, (gpio_num_t)STEPPER_IN4_GPIO);
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    // 供文件底部 extern "C" 接口判断 OLED 是否初始化成功
    bool OledReady() const { return has_oled_; }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    /**
     * @brief 获取步进电机控制器
     * @return 步进电机控制器指针
     */
    StepperMotorController* GetStepperMotor() {
        return stepper_motor_;
    }
};

// ── 屏幕「文字 / 表情」模式：给 web_server 的弱符号落点 ──────────────────
//   web_server 是公共模块，不方便 include oled_display.h 依赖具体屏幕，
//   所以用 extern "C" 把能力暴露出去，网页 /api/display_mode 才能读写。
extern "C" void SetChatDisplayMode(bool text_mode) {
    auto* board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto* oled = static_cast<OledDisplay*>(board->GetDisplay());
    if (!board->OledReady() || !oled) return;
    oled->SetChatDisplayMode(text_mode);
}

extern "C" bool IsChatTextMode() {
    auto* board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto* oled = static_cast<OledDisplay*>(board->GetDisplay());
    return (board->OledReady() && oled) ? oled->IsChatTextMode() : false;
}

// ── 待机壁纸（0..OledDisplay::kWallpaperCount-1）：给 web_server 的弱符号落点 ──
//   网页 /api/theme 切换时同步写入 NVS，重启后由 InitializeSsd1306Display 恢复。
extern "C" void SetWallpaper(int wp) {
    auto* board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto* oled = static_cast<OledDisplay*>(board->GetDisplay());
    if (!board->OledReady() || !oled) return;
    oled->SetWallpaper(wp);
    Settings("display", true).SetInt("wallpaper", oled->GetWallpaper());
}

extern "C" int GetWallpaper() {
    auto* board = static_cast<CompactWifiBoard*>(&Board::GetInstance());
    auto* oled = static_cast<OledDisplay*>(board->GetDisplay());
    return (board->OledReady() && oled) ? oled->GetWallpaper() : 0;
}

// 壁纸总数：网页按钮数量 / 范围校验都以它为准，新增效果不用改 web_server.cc
extern "C" int GetWallpaperCount() {
    return OledDisplay::kWallpaperCount;
}

// 壁纸中文名：日志和网页共用一份（越界返回 "?"）
extern "C" const char* GetWallpaperName(int wp) {
    return OledDisplay::WallpaperName(wp);
}

DECLARE_BOARD(CompactWifiBoard);
