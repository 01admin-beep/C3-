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
        //   1) NVS 里上次扫到的引脚（秒连）
        //   2) config.h 默认 + 常见板型组合
        //   3) 全 GPIO 组合扫描（约 20 秒，扫到后写入 NVS，下次不再慢扫）
        static const uint8_t kAddrs[] = {0x3C, 0x3D};
        Settings display_settings("display");

        auto try_pair = [this](int sda, int scl) -> int {
            if (sda < 0 || scl < 0 || sda == scl) return -1;
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
            if (i2c_new_master_bus(&bus_config, &bus) != ESP_OK) return -1;
            int found = -1;
            for (uint8_t addr : kAddrs) {
                if (i2c_master_probe(bus, addr, 20) == ESP_OK) { found = addr; break; }
            }
            if (found < 0) {
                i2c_del_master_bus(bus);
                return -1;
            }
            display_i2c_bus_ = bus;
            return found;
        };

        auto save_pins = [](int sda, int scl) {
            Settings("display", true).SetInt("i2c_pins", (sda << 8) | scl);
        };

        // 1) NVS 缓存
        int saved = display_settings.GetInt("i2c_pins", -1);
        if (saved > 0) {
            int addr = try_pair(saved >> 8, saved & 0xFF);
            if (addr >= 0) {
                display_addr_ = (uint8_t)addr;
                ESP_LOGI(TAG, "OLED found (NVS): SDA=%d SCL=%d addr=0x%02X", saved >> 8, saved & 0xFF, addr);
                return;
            }
        }

        // 2) 常见组合（仅限安全引脚；本板 OLED 实测在 SDA=2/SCL=1）
        //    禁止出现电机(8/19/20/3)/音频(4-7/15/16)/灯(18)/按键(39/40/47)引脚
        static const int kCommon[][2] = {
            {DISPLAY_SDA_PIN, DISPLAY_SCL_PIN},
            {2, 1}, {1, 2},
            {41, 42}, {12, 13}, {13, 12}, {9, 10}, {10, 11},
            {11, 12}, {14, 21}, {21, 14}, {38, 41}, {41, 38},
        };
        for (const auto& pins : kCommon) {
            int addr = try_pair(pins[0], pins[1]);
            if (addr >= 0) {
                display_addr_ = (uint8_t)addr;
                save_pins(pins[0], pins[1]);
                ESP_LOGI(TAG, "OLED found: SDA=%d SCL=%d addr=0x%02X", pins[0], pins[1], addr);
                return;
            }
        }

        // 3) 安全引脚组合扫描
        //    只扫无冲突引脚，显式排除（在这些脚上发探测波形会干扰外设）：
        //    - 音频 I2S：4/5/6/7/15/16
        //    - 步进电机 IN1-IN4：8/19/20/3（IN2/IN3 在 USB D-/D+ 上，乱扫会让轮子乱转！）
        //    - 台灯：18；按键：0/39/40/47；LED：48
        //    - UART0：43/44；strapping：45/46；flash/PSRAM：26-37
        //    （28BYJ-48 一共只需 4 根信号线，剩余安全脚足够覆盖 OLED 的接线）
        static const int kScanPins[] = {
            1, 2, 9, 10, 11, 12, 13, 14, 17, 21, 38,
        };
        const int n = sizeof(kScanPins) / sizeof(kScanPins[0]);
        ESP_LOGI(TAG, "Full I2C pin scan started (%d pairs)...", n * (n - 1));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i == j) continue;
                int addr = try_pair(kScanPins[i], kScanPins[j]);
                if (addr >= 0) {
                    display_addr_ = (uint8_t)addr;
                    save_pins(kScanPins[i], kScanPins[j]);
                    ESP_LOGI(TAG, "OLED found (scan): SDA=%d SCL=%d addr=0x%02X", kScanPins[i], kScanPins[j], addr);
                    return;
                }
            }
        }
        ESP_LOGE(TAG, "OLED not found on any GPIO pair, display disabled");
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
