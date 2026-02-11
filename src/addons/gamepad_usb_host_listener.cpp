#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"
#include <cstring>

// Solo Drivers de PlayStation
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

// --- CONFIGURACIÓN DE MACROS ---
#define ANTI_RECOIL_STRENGTH 1200 

// --- COLORES (FUERZA ROJO Y BLANCO, CERO AZUL) ---
#define LED_EAFC_R     0xFF 
#define LED_EAFC_G     0x00
#define LED_EAFC_B     0x00 

#define LED_WARZONE_R  0xFF
#define LED_WARZONE_G  0xFF
#define LED_WARZONE_B  0xFF

// --- ESTRUCTURA DS5 (DualSense) ---
#pragma pack(push, 1)
struct DS5OutReport {
    uint8_t report_id;          // Byte 0: 0x02
    uint8_t valid_flag0;        // Byte 1: 0xFF para forzar prioridad
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

// Flags de Control Protocolo Sony
#define DS_FLAG1_LIGHTBAR_ENABLE       (1 << 2) 
#define DS_FLAG1_PLAYER_LED_ENABLE     (1 << 4)
#define DS_FLAG2_LIGHTBAR_SETUP_ENABLE (1 << 1)
#define DS_LIGHTBAR_SETUP_LIGHT_ON     (1 << 0)

enum Profile { PROFILE_EAFC, PROFILE_WARZONE };
static Profile current_profile = PROFILE_EAFC; 
static uint32_t profile_switch_timer = 0;
static bool profile_switch_held = false;

// Banderas de estado para el LED
static bool ds5_led_needs_update = true;
static bool ds5_pending_init = false;

// Variables de macros (Mute y Turbo)
static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;
static uint32_t turbo_timer = 0;
static bool turbo_state = false;

// --------------------------------------------------------------------------------
//                                  MÉTODOS CORE
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::setup() {
    _controller_host_enabled = false;
    ds5_pending_init = false;
}

void GamepadUSBHostListener::process() {
    // 1. Inicialización diferida (Fuera del callback para evitar bloqueos)
    if (_controller_host_enabled && ds5_pending_init) {
        init_ds5_led(_controller_dev_addr, _controller_instance);
        ds5_pending_init = false; 
    }

    // 2. Reintento de actualización de LED (Si el USB estaba ocupado antes)
    if (_controller_host_enabled && ds5_led_needs_update) {
        update_ds5();
    }

    Gamepad *gamepad = Storage::getInstance().GetGamepad();
    
    // Pasar estados del mando host al mando emulado
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

    if (controller_pid == 0x0CE6) { // DualSense
        isDS4Identified = true;
        ds5_pending_init = true;
        ds5_led_needs_update = true;
    } else {
        isDS4Identified = false;
        init_ds4(desc_report, desc_len);
    }
}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    isDS4Identified = false;
    ds5_pending_init = false;
}

// --------------------------------------------------------------------------------
//                          GESTIÓN DE LED PS5
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    DS5OutReport out_report;
    std::memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;
    out_report.valid_flag0 = 0xFF; // Prioridad máxima sobre el sistema
    out_report.valid_flag1 = DS_FLAG1_LIGHTBAR_ENABLE | DS_FLAG1_PLAYER_LED_ENABLE;
    out_report.valid_flag2 = DS_FLAG2_LIGHTBAR_SETUP_ENABLE;
    out_report.lightbar_setup = DS_LIGHTBAR_SETUP_LIGHT_ON;
    out_report.led_brightness = 0xFF;
    out_report.player_leds = 0x00; // Apagar puntitos blancos para limpiar el color

    out_report.lightbar_red   = LED_EAFC_R; 
    out_report.lightbar_green = LED_EAFC_G;
    out_report.lightbar_blue  = LED_EAFC_B;

    // Intentar enviar con pequeña espera para asegurar el canal
    int reintentos = 0;
    while (!tuh_hid_send_report(dev_addr, instance, 0, &out_report, sizeof(DS5OutReport)) && reintentos < 50) {
        tuh_task();
        reintentos++;
    }

    tuh_hid_receive_report(dev_addr, instance);
    ds5_led_needs_update = false;
}

void GamepadUSBHostListener::update_ds5() {
    DS5OutReport out_report;
    std::memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;
    out_report.valid_flag0 = 0xFF; 
    out_report.valid_flag1 = DS_FLAG1_LIGHTBAR_ENABLE | DS_FLAG1_PLAYER_LED_ENABLE;
    out_report.valid_flag2 = DS_FLAG2_LIGHTBAR_SETUP_ENABLE;
    out_report.lightbar_setup = DS_LIGHTBAR_SETUP_LIGHT_ON;
    out_report.led_brightness = 0xFF;
    out_report.player_leds = 0x00;

    if (current_profile == PROFILE_EAFC) {
        out_report.lightbar_red   = LED_EAFC_R; // ROJO
        out_report.lightbar_green = 0x00;
        out_report.lightbar_blue  = 0x00;
    } else {
        out_report.lightbar_red   = LED_WARZONE_R; // BLANCO
        out_report.lightbar_green = LED_WARZONE_G;
        out_report.lightbar_blue  = LED_WARZONE_B;
    }

    // Intento no bloqueante
    if (tuh_hid_send_report(_controller_dev_addr, _controller_instance, 0, &out_report, sizeof(DS5OutReport))) {
        ds5_led_needs_update = false; 
    }
}

// --------------------------------------------------------------------------------
//                          LÓGICA DE PROCESAMIENTO PS5
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    if (report[0] != 1) return;

    DSReport controller_report;
    memcpy(&controller_report, report, sizeof(controller_report));

    // --- CAMBIO DE PERFIL (Select + Start por 1 segundo) ---
    if (controller_report.buttonSelect && controller_report.buttonStart) {
        if (!profile_switch_held) {
            profile_switch_held = true;
            profile_switch_timer = getMillis();
        } else if (getMillis() - profile_switch_timer > 1000) {
            current_profile = (current_profile == PROFILE_EAFC) ? PROFILE_WARZONE : PROFILE_EAFC;
            ds5_led_needs_update = true; 
            profile_switch_timer = getMillis() + 3000; // Cooldown
        }
    } else {
        profile_switch_held = false;
    }

    // Mapeo sticks
    _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
    _controller_host_state.buttons = 0;

    // --- PERFIL 1: EAFC (ROJO) ---
    if (current_profile == PROFILE_EAFC) {
        if (controller_report.buttonHome && !macro_mute_active) {
            macro_mute_active = true;
            macro_mute_start_time = getMillis();
        }
        if (macro_mute_active) {
            if (getMillis() - macro_mute_start_time < 480) {
                _controller_host_state.buttons |= GAMEPAD_MASK_B3; // Cuadrado
                _controller_host_state.buttons |= GAMEPAD_MASK_B2; // Cruz
            } else macro_mute_active = false;
        }
        if (controller_report.buttonR1) _controller_host_state.rt = 255;
        if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
    } 
    // --- PERFIL 2: WARZONE (BLANCO) ---
    else {
        if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
            uint32_t recoil = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;
            _controller_host_state.ry = (recoil > GAMEPAD_JOYSTICK_MAX) ? GAMEPAD_JOYSTICK_MAX : recoil;
        }
        if (controller_report.buttonL1) {
            if (getMillis() - turbo_timer > 40) {
                turbo_state = !turbo_state;
                turbo_timer = getMillis();
            }
            if (turbo_state) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        } else turbo_state = false;
        
        if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
    }

    // Botones universales
    if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
    if (controller_report.buttonEast)  _controller_host_state.buttons |= GAMEPAD_MASK_B2;
    if (controller_report.buttonWest)  _controller_host_state.buttons |= GAMEPAD_MASK_B3;
    if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
}

// --------------------------------------------------------------------------------
//                              STUBS Y HELPERS
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if (!_controller_host_enabled) return;
    process_ctrlr_report(dev_addr, report, len);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    if (controller_pid == 0x0CE6) process_ds(report, len);
    else if (isDS4Identified) process_ds4(report, len);
}

uint32_t GamepadUSBHostListener::map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void GamepadUSBHostListener::update_ctrlr() {} 
void GamepadUSBHostListener::update_ds4() {}
bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) { return false; }
bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) { return false; }
void GamepadUSBHostListener::set_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {}
void GamepadUSBHostListener::get_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {}
void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {}
void GamepadUSBHostListener::init_ds4(const uint8_t* descReport, uint16_t descLen) {}
void GamepadUSBHostListener::setup_ds4() {}
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
