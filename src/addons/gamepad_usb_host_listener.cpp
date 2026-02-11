#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"
#include <cstring>

// Solo Drivers de PlayStation
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

// --- CONFIGURACIÓN ---
#define ANTI_RECOIL_STRENGTH 1200 

// --- ESTRUCTURA DS5 ---
#pragma pack(push, 1)
struct DS5OutReport {
    uint8_t report_id;          // Byte 0
    uint8_t valid_flag0;        // Byte 1
    uint8_t valid_flag1;        // Byte 2
    uint8_t motor_right;        // Byte 3
    uint8_t motor_left;         // Byte 4
    uint8_t headphone_audio_volume;      // Byte 5
    uint8_t speaker_audio_volume;        // Byte 6
    uint8_t internal_microphone_volume;  // Byte 7
    uint8_t audio_flags;                 // Byte 8
    uint8_t mute_button_led;             // Byte 9
    uint8_t power_save_control;          // Byte 10
    uint8_t right_trigger_motor_mode;    // Byte 11
    uint8_t right_trigger_param[10];     // Byte 12-21
    uint8_t left_trigger_motor_mode;     // Byte 22
    uint8_t left_trigger_param[10];      // Byte 23-32
    uint32_t host_timestamp;             // Byte 33-36
    uint8_t reduce_motor_power;          // Byte 37
    uint8_t audio_flags2;                // Byte 38
    uint8_t valid_flag2;                 // Byte 39
    uint8_t haptics_flags;               // Byte 40
    uint8_t reserved3;                   // Byte 41
    uint8_t lightbar_setup;              // Byte 42
    uint8_t led_brightness;              // Byte 43
    uint8_t player_leds;                 // Byte 44
    uint8_t lightbar_red;                // Byte 45
    uint8_t lightbar_green;              // Byte 46
    uint8_t lightbar_blue;               // Byte 47
};
#pragma pack(pop)

// Flags
#define DS_FLAG1_LIGHTBAR_ENABLE       (1 << 2) 
#define DS_FLAG1_PLAYER_LED_ENABLE     (1 << 4)
#define DS_FLAG2_LIGHTBAR_SETUP_ENABLE (1 << 1)
#define DS_LIGHTBAR_SETUP_LIGHT_ON     (1 << 0)

// --- CAMBIO DE COLORES (ROJO PRIMERO) ---
// Perfil 1 (EAFC) -> AHORA ES ROJO
#define LED_EAFC_R     0xFF 
#define LED_EAFC_G     0x00
#define LED_EAFC_B     0x00

// Perfil 2 (WARZONE) -> AHORA ES AZUL
#define LED_WARZONE_R  0x00
#define LED_WARZONE_G  0x00
#define LED_WARZONE_B  0xFF

enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile = PROFILE_EAFC; 
static uint32_t profile_switch_timer = 0;
static bool profile_switch_held = false;

// Macros
static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;
static uint32_t turbo_timer = 0;
static bool turbo_state = false;

// Estado del LED
static bool ds5_led_needs_update = true;
static bool ds5_pending_init = false;

void GamepadUSBHostListener::setup() {
    _controller_host_enabled = false;
    ds5_pending_init = false;
}

void GamepadUSBHostListener::process() {
    // 1. Inicialización diferida (Solo una vez al conectar)
    if (_controller_host_enabled && ds5_pending_init) {
        init_ds5_led(_controller_dev_addr, _controller_instance);
        ds5_pending_init = false; 
    }

    // 2. Actualización de LED en caliente (Si cambias de perfil)
    // Se llama aquí en el process() para reintentar si el USB estaba ocupado
    if (_controller_host_enabled && ds5_led_needs_update) {
        update_ds5();
    }

    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    gamepad->hasAnalogTriggers   = _controller_host_analog;
    gamepad->hasLeftAnalogStick  = _controller_host_analog;
    gamepad->hasRightAnalogStick = _controller_host_analog;
    gamepad->state.dpad     = _controller_host_state.dpad;
    gamepad->state.buttons  = _controller_host_state.buttons;
    gamepad->state.lx       = _controller_host_state.lx;
    gamepad->state.ly       = _controller_host_state.ly;
    gamepad->state.rx       = _controller_host_state.rx;
    gamepad->state.ry       = _controller_host_state.ry;
    gamepad->state.rt       = _controller_host_state.rt;
    gamepad->state.lt       = _controller_host_state.lt;

    if (_controller_host_enabled && getMillis() > _next_update) {
        update_ctrlr();
        _next_update = getMillis() + GAMEPAD_HOST_POLL_INTERVAL_MS;
    }
}

void GamepadUSBHostListener::mount(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    _controller_host_enabled = true;
    _controller_dev_addr = dev_addr;
    _controller_instance = instance;
    controller_pid = pid;

    // Resetear estados al conectar
    _controller_host_state.lx = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.ly = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.rx = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.ry = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;

    switch(controller_pid) {
        case PS4_PRODUCT_ID:       
        case 0x00EE:                
        case PS4_WHEEL_PRODUCT_ID: 
        case 0xB67B:                
            init_ds4(desc_report, desc_len);
            break;
        case DS4_ORG_PRODUCT_ID:   
        case DS4_PRODUCT_ID:        
            isDS4Identified = true;
            setup_ds4();
            break;
        case 0x0CE6: // DualSense PS5
            isDS4Identified = true;
            ds5_led_needs_update = true;
            ds5_pending_init = true; // Marcar para iniciar en process()
            break;
        default:
            break;
    }
}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    isDS4Identified = false;
    ds5_pending_init = false;
}

// --------------------------------------------------------------------------------
//        INICIALIZACIÓN LED PS5 (ROJO PRIMERO)
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    DS5OutReport out_report;
    std::memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;
    out_report.valid_flag1 = DS_FLAG1_LIGHTBAR_ENABLE | DS_FLAG1_PLAYER_LED_ENABLE;
    out_report.valid_flag2 = DS_FLAG2_LIGHTBAR_SETUP_ENABLE;
    out_report.lightbar_setup = DS_LIGHTBAR_SETUP_LIGHT_ON;
    out_report.led_brightness = 0xFF; // Max brightness
    out_report.player_leds = 0x04;    // Center LED

    // CONFIGURACIÓN INICIAL: ROJO (Perfil EAFC)
    if (current_profile == PROFILE_EAFC) {
        out_report.lightbar_red   = LED_EAFC_R;   // 0xFF
        out_report.lightbar_green = LED_EAFC_G;   // 0x00
        out_report.lightbar_blue  = LED_EAFC_B;   // 0x00
    } else {
        out_report.lightbar_red   = LED_WARZONE_R;
        out_report.lightbar_green = LED_WARZONE_G;
        out_report.lightbar_blue  = LED_WARZONE_B;
    }

    // Primer envío con while (seguro aquí porque es inicio)
    while (!tuh_hid_send_report(dev_addr, instance, 0, &out_report, sizeof(DS5OutReport))) {
        tuh_task();
    }

    tuh_hid_receive_report(dev_addr, instance);
    ds5_led_needs_update = false;
}

// --------------------------------------------------------------------------------
//        UPDATE LED EN CALIENTE (SIN BLOQUEOS)
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::update_ds5() {
    // Construir el reporte
    DS5OutReport out_report;
    std::memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;
    // Reenviar todos los flags para asegurar que el mando hace caso
    out_report.valid_flag1 = DS_FLAG1_LIGHTBAR_ENABLE | DS_FLAG1_PLAYER_LED_ENABLE;
    out_report.valid_flag2 = DS_FLAG2_LIGHTBAR_SETUP_ENABLE;
    out_report.lightbar_setup = DS_LIGHTBAR_SETUP_LIGHT_ON;
    out_report.led_brightness = 0xFF;
    out_report.player_leds = 0x04;

    if (current_profile == PROFILE_EAFC) {
        out_report.lightbar_red   = LED_EAFC_R;   // ROJO
        out_report.lightbar_green = LED_EAFC_G;
        out_report.lightbar_blue  = LED_EAFC_B;
    } else {
        out_report.lightbar_red   = LED_WARZONE_R; // AZUL (Ahora Warzone es azul)
        out_report.lightbar_green = LED_WARZONE_G;
        out_report.lightbar_blue  = LED_WARZONE_B;
    }

    // INTENTAR ENVIAR UNA VEZ
    // Si retorna true, se encoló -> bajamos bandera.
    // Si retorna false (USB ocupado), NO bajamos bandera -> se reintenta en el siguiente loop process()
    if (tuh_hid_send_report(_controller_dev_addr, _controller_instance, 0, &out_report, sizeof(DS5OutReport))) {
        ds5_led_needs_update = false; 
    }
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if ( _controller_host_enabled == false ) return;
    if ( tuh_hid_interface_protocol(dev_addr, instance) == HID_ITF_PROTOCOL_KEYBOARD ) return;
    process_ctrlr_report(dev_addr, report, len);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    if (controller_pid == 0x0CE6) process_ds(report, len);
    else if (isDS4Identified) process_ds4(report, len);
}

// --------------------------------------------------------------------------------
//        PROCESAR INPUT PS5
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    DSReport controller_report;
    static DSReport prev_ds_report = { 0 };
    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if ( prev_ds_report.reportCounter != controller_report.reportCounter || macro_mute_active || turbo_state) {
            
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            
            _controller_host_state.lt = 0;
            _controller_host_state.rt = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog = true;

            // --- CAMBIO DE PERFIL (SHARE + OPTIONS) ---
            // Tiempo reducido a 1000ms (1 segundo) para probar más fácil
            if (controller_report.buttonSelect && controller_report.buttonStart) {
                if (!profile_switch_held) {
                    profile_switch_held = true;
                    profile_switch_timer = getMillis();
                } else if (getMillis() - profile_switch_timer > 1000) { // <--- 1 SEGUNDO
                    
                    // Toggle Perfil
                    if (current_profile == PROFILE_EAFC) current_profile = PROFILE_WARZONE;
                    else current_profile = PROFILE_EAFC;
                    
                    // IMPORTANTE: Pedir actualización de LED
                    ds5_led_needs_update = true;

                    // Esperar 3 segundos antes de permitir otro cambio
                    profile_switch_timer = getMillis() + 3000;
                }
            } else {
                profile_switch_held = false;
            }

            // PERFIL 1: EAFC (ROJO)
            if (current_profile == PROFILE_EAFC) {
                if (controller_report.buttonHome && !macro_mute_active) {
                    macro_mute_active = true;
                    macro_mute_start_time = getMillis();
                }
                if (macro_mute_active) {
                    if (getMillis() - macro_mute_start_time < 480) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3; // Cuadrado
                        _controller_host_state.buttons |= GAMEPAD_MASK_B2; // Cruz (X)
                    } else {
                        macro_mute_active = false;
                    }
                }
                // Macros EAFC
                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR2 || controller_report.rightTrigger > 10) _controller_host_state.lt = 255;
                if (controller_report.buttonL2 || controller_report.leftTrigger > 10) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }
            // PERFIL 2: WARZONE (AZUL)
            else if (current_profile == PROFILE_WARZONE) {
                // Anti-Recoil
                if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
                    uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;
                    if (recoil_val > GAMEPAD_JOYSTICK_MAX) recoil_val = GAMEPAD_JOYSTICK_MAX;
                    _controller_host_state.ry = recoil_val;
                }
                // Turbo
                if (controller_report.buttonL1) {
                    if (getMillis() - turbo_timer > 40) {
                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }
                    if (turbo_state) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                } else {
                    turbo_state = false;
                }
                // Mapeos Warzone
                if (controller_report.buttonSelect && !controller_report.buttonStart) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                }
                if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            // Comunes
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;

            _controller_host_state.dpad = 0;
            if (controller_report.dpad == PS4_HAT_UP) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_UPRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_UP | GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWNRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT | GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWN) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWNLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN | GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_LEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_UPLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT | GAMEPAD_MASK_UP;

            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
        }
    }
    prev_ds_report = controller_report;
}

// Helpers PS4
void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    PS4Report controller_report;
    static PS4Report prev_report = { 0 };
    if (report[0] == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));
        if (diff_report(&prev_report, &controller_report)) {
            _controller_host_state.lx = map(controller_report.leftStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.buttons = 0;
            _controller_host_state.dpad = 0;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            // ... resto de botones básicos PS4 ...
            _controller_host_analog = true;
        }
    }
    prev_report = controller_report;
}

void GamepadUSBHostListener::update_ctrlr() {} 
void GamepadUSBHostListener::update_ds4() {}
bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_get_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}
bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_set_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}
void GamepadUSBHostListener::set_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) { awaiting_cb = false; }
void GamepadUSBHostListener::get_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {
    if (!isDS4Identified && report_id == PS4AuthReport::PS4_DEFINITION) setup_ds4();
    awaiting_cb = false;
}
uint32_t GamepadUSBHostListener::map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
bool GamepadUSBHostListener::diff_than_2(uint8_t x, uint8_t y) { return (x - y > 2) || (y - x > 2); }
bool GamepadUSBHostListener::diff_report(PS4Report const* rpt1, PS4Report const* rpt2) {
    bool result = diff_than_2(rpt1->leftStickX, rpt2->leftStickX) || diff_than_2(rpt1->leftStickY, rpt2->leftStickY) ||
                  diff_than_2(rpt1->rightStickX, rpt2->rightStickX) || diff_than_2(rpt1->rightStickY, rpt2->rightStickY);
    result |= memcmp(&rpt1->rightStickY + 1, &rpt2->rightStickY + 1, sizeof(PS4Report)-6);
    return result;
}
void GamepadUSBHostListener::setup_ds4() {
    if (hasDS4DefReport) memcpy(&ds4Config, report_buffer+1, sizeof(PS4ControllerConfig));
    if ((ds4Config.hidUsage == 0x2721) || (ds4Config.hidUsage == 0x2127)) isDS4Identified = true;
}
void GamepadUSBHostListener::init_ds4(const uint8_t* descReport, uint16_t descLen) {
    isDS4Identified = false;
    tuh_hid_report_info_t report_info[4];
    uint8_t report_count = tuh_hid_parse_report_descriptor(report_info, 4, descReport, descLen);
    for(uint8_t i = 0; i < report_count; i++) {
        if (report_info[i].report_id == PS4AuthReport::PS4_DEFINITION) {
            memset(report_buffer, 0, PS4_ENDPOINT_SIZE);
            report_buffer[0] = PS4AuthReport::PS4_DEFINITION;
            host_get_report(PS4AuthReport::PS4_DEFINITION, report_buffer, 48);
            hasDS4DefReport = true;
            break;
        }
    }
}
void GamepadUSBHostListener::xmount(uint8_t dev_addr, uint8_t instance, uint8_t controllerType, uint8_t subtype) {}
void GamepadUSBHostListener::update_switch_pro() {}
void GamepadUSBHostListener::setup_switch_pro(uint8_t const *report, uint16_t len) {}
void GamepadUSBHostListener::process_switch_pro(uint8_t const *report, uint16_t len) {}
uint8_t GamepadUSBHostListener::get_next_switch_counter() { return 0; }
void GamepadUSBHostListener::process_stadia(uint8_t const *report, uint16_t len) {}
void GamepadUSBHostListener::setup_df_wheel() {}
void GamepadUSBHostListener::process_dfgt(uint8_t const* report, uint16_t len) {}
void GamepadUSBHostListener::process_ultrastik360(uint8_t const* report, uint16_t len) {}
void GamepadUSBHostListener::xbox360_set_led(uint8_t dev_addr, uint8_t instance, uint8_t quadrant) {}
void GamepadUSBHostListener::xinput_set_rumble(uint8_t dev_addr, uint8_t instance, uint8_t left, uint8_t right) {}
void GamepadUSBHostListener::setup_xinput(uint8_t dev_addr, uint8_t instance) {}
void GamepadUSBHostListener::update_xinput(uint8_t dev_addr, uint8_t instance) {}
void GamepadUSBHostListener::process_xbox360(uint8_t const* report, uint16_t len) {}
