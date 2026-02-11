#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"

// Solo Drivers de PlayStation
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

// --- CONFIGURACIÓN ---
#define ANTI_RECOIL_STRENGTH 1200 

// --- ESTRUCTURA DEL OUT REPORT PARA PS5 (DualSense) ---
// Basado en el protocolo USB HID del DualSense
#pragma pack(push, 1)
struct DS5OutReport {
    uint8_t report_id;          // 0x02 para USB
    uint8_t control_flag[2];    // [0]=0x02, [1]=0x02 para habilitar motor+LED
    uint8_t motor_right;        // Rumble derecho (alta frecuencia)
    uint8_t motor_left;         // Rumble izquierdo (baja frecuencia)
    uint8_t reserved1[4];       // Headphone, speaker volume, mic volume, audio flags
    uint8_t mute_button_led;    // LED del botón mute
    uint8_t reserved2[7];       // Power reduction, padding
    uint8_t reserved3[5];       // Right trigger effect
    uint8_t reserved4[5];       // Left trigger effect  
    uint8_t reserved5[6];       // Padding
    uint8_t led_control_flag;   // 0x01=mic mute LED, 0x02=lightbar control, 0x04=player LEDs
    uint8_t reserved6[2];       // Padding
    uint8_t pulse_option;       // LED fade animation
    uint8_t led_brightness;     // Brillo: 0x00=max, 0x01=mid, 0x02=low
    uint8_t player_number;      // Player LED (1-5)
    uint8_t lightbar_red;       // Lightbar R (0-255)
    uint8_t lightbar_green;     // Lightbar G (0-255)
    uint8_t lightbar_blue;      // Lightbar B (0-255)
};
#pragma pack(pop)

// Colores por perfil
#define LED_EAFC_R     0x00
#define LED_EAFC_G     0x00
#define LED_EAFC_B     0xFF    // Azul para EA FC

#define LED_WARZONE_R  0xFF
#define LED_WARZONE_G  0x00
#define LED_WARZONE_B  0x00    // Rojo para Warzone

enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile = PROFILE_EAFC; 
static uint32_t profile_switch_timer = 0;
static bool profile_switch_held = false;

// Variables Macro Mute
static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;

// Variables Macro Turbo
static uint32_t turbo_timer = 0;
static bool turbo_state = false;

// Variable para saber si necesitamos actualizar el LED del PS5
static bool ds5_led_needs_update = true;

void GamepadUSBHostListener::setup() {
    _controller_host_enabled = false;
#if GAMEPAD_HOST_DEBUG
    stdio_init_all();
#endif
}

void GamepadUSBHostListener::process() {
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

// --------------------------------------------------------------------------------
//                                  INICIALIZACIÓN
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::mount(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    uint16_t vid = 0;
    uint16_t pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    _controller_host_enabled = true;
    _controller_dev_addr = dev_addr;
    _controller_instance = instance;
    _controller_type = 0;
    controller_vid = vid;
    controller_pid = pid;

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.lx = joystick_mid;
    _controller_host_state.ly = joystick_mid;
    _controller_host_state.rx = joystick_mid;
    _controller_host_state.ry = joystick_mid;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;

    switch(controller_pid)
    {
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
            // Encender LED inmediatamente al conectar
            init_ds5_led(dev_addr, instance);
            break;
        default:
            break;
    }
}

void GamepadUSBHostListener::xmount(uint8_t dev_addr, uint8_t instance, uint8_t controllerType, uint8_t subtype) {}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    controller_pid = 0x00;
    controller_vid = 0x00;
    _controller_dev_addr = 0;
    _controller_instance = 0;
    _controller_type = 0;
    isDS4Identified = false;
    hasDS4DefReport = false;
    ds5_led_needs_update = true;
}

// --------------------------------------------------------------------------------
//                    INICIALIZACIÓN LED PS5 (DualSense)
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    // Primer envío: configurar lightbar + player LED + brillo
    DS5OutReport out_report;
    memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;        // Report ID para USB
    out_report.control_flag[0] = 0x02;  // Habilitar control de motor
    out_report.control_flag[1] = 0x02;  // Habilitar control de LED
    out_report.led_control_flag = 0x01 | 0x02 | 0x04; // Mic LED + Lightbar + Player LEDs
    out_report.pulse_option = 0x01;     // Sin parpadeo
    out_report.led_brightness = 0x02;   // Brillo máximo (0x00=max, pero 0x02 funciona mejor)
    out_report.player_number = 0x04;    // Player 1 LED encendido (centro)

    // Color según perfil actual
    if (current_profile == PROFILE_EAFC) {
        out_report.lightbar_red   = LED_EAFC_R;
        out_report.lightbar_green = LED_EAFC_G;
        out_report.lightbar_blue  = LED_EAFC_B;
    } else {
        out_report.lightbar_red   = LED_WARZONE_R;
        out_report.lightbar_green = LED_WARZONE_G;
        out_report.lightbar_blue  = LED_WARZONE_B;
    }

    tuh_hid_send_report(dev_addr, instance, 0, &out_report, sizeof(DS5OutReport));
}

// --------------------------------------------------------------------------------
//                                  REPORTES
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if ( _controller_host_enabled == false ) return;
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    if ( itf_protocol == HID_ITF_PROTOCOL_KEYBOARD ) return;
    process_ctrlr_report(dev_addr, report, len);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    switch(controller_pid)
    {
        case DS4_ORG_PRODUCT_ID:   
        case DS4_PRODUCT_ID:       
        case PS4_PRODUCT_ID:       
        case PS4_WHEEL_PRODUCT_ID: 
        case 0xB67B:               
        case 0x00EE:               
            if (isDS4Identified) process_ds4(report, len);
            break;
        case 0x0CE6: // DualSense
            process_ds(report, len);
            break;
        default:
            break;
    }
}

// --------------------------------------------------------------------------------
//                                  LÓGICA DEL MANDO PS4
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    PS4Report controller_report;
    static PS4Report prev_report = { 0 };
    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if ( diff_report(&prev_report, &controller_report) || macro_mute_active || turbo_state) {
            
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            
            _controller_host_state.lt = 0;
            _controller_host_state.rt = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog = true;

            // --- CAMBIO DE PERFIL ---
            if (controller_report.buttonSelect && controller_report.buttonStart) {
                if (!profile_switch_held) {
                    profile_switch_held = true;
                    profile_switch_timer = getMillis();
                } else if (getMillis() - profile_switch_timer > 2000) {
                    if (current_profile == PROFILE_EAFC) current_profile = PROFILE_WARZONE;
                    else current_profile = PROFILE_EAFC;
                    
                    profile_switch_timer = getMillis() + 5000;
                }
            } else {
                profile_switch_held = false;
            }

            // --- PERFIL 1: EA FC 26 ---
            if (current_profile == PROFILE_EAFC) {
                if (controller_report.buttonHome && !macro_mute_active) {
                    macro_mute_active = true;
                    macro_mute_start_time = getMillis();
                }
                if (macro_mute_active) {
                    if (getMillis() - macro_mute_start_time < 480) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                        _controller_host_state.buttons |= GAMEPAD_MASK_B2;
                    } else {
                        macro_mute_active = false;
                    }
                }

                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR2 || controller_report.rightTrigger > 10) _controller_host_state.lt = 255;
                if (controller_report.buttonL2 || controller_report.leftTrigger > 10) _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            // --- PERFIL 2: WARZONE ---
            else if (current_profile == PROFILE_WARZONE) {
                if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
                    uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;
                    if (recoil_val > GAMEPAD_JOYSTICK_MAX) recoil_val = GAMEPAD_JOYSTICK_MAX;
                    _controller_host_state.ry = recoil_val;
                }

                if (controller_report.buttonL1) {
                    if (getMillis() - turbo_timer > 40) {
                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }
                    if (turbo_state) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                } else {
                    turbo_state = false;
                }

                if (controller_report.buttonSelect && !controller_report.buttonStart) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                }

                if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            // --- COMUNES ---
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
    prev_report = controller_report;
}

// --------------------------------------------------------------------------------
//                                  LÓGICA DEL MANDO PS5 (DualSense)
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    DSReport controller_report;
    static DSReport prev_ds_report = { 0 };
    static Profile prev_profile_for_led = PROFILE_EAFC;
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

            // --- CAMBIO DE PERFIL ---
            if (controller_report.buttonSelect && controller_report.buttonStart) {
                if (!profile_switch_held) {
                    profile_switch_held = true;
                    profile_switch_timer = getMillis();
                } else if (getMillis() - profile_switch_timer > 2000) {
                    if (current_profile == PROFILE_EAFC) current_profile = PROFILE_WARZONE;
                    else current_profile = PROFILE_EAFC;
                    profile_switch_timer = getMillis() + 5000;
                    // Marcar que el LED necesita actualizarse al cambiar perfil
                    ds5_led_needs_update = true;
                }
            } else {
                profile_switch_held = false;
            }

            // Detectar cambio de perfil para actualizar LED
            if (prev_profile_for_led != current_profile) {
                ds5_led_needs_update = true;
                prev_profile_for_led = current_profile;
            }

            // --- PERFIL 1: EA FC 26 ---
            if (current_profile == PROFILE_EAFC) {
                if (controller_report.buttonHome && !macro_mute_active) {
                    macro_mute_active = true;
                    macro_mute_start_time = getMillis();
                }
                if (macro_mute_active) {
                    if (getMillis() - macro_mute_start_time < 480) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                        _controller_host_state.buttons |= GAMEPAD_MASK_B2;
                    } else {
                        macro_mute_active = false;
                    }
                }

                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR2 || controller_report.rightTrigger > 10) _controller_host_state.lt = 255;
                if (controller_report.buttonL2 || controller_report.leftTrigger > 10) _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            // --- PERFIL 2: WARZONE ---
            else if (current_profile == PROFILE_WARZONE) {
                if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
                    uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;
                    if (recoil_val > GAMEPAD_JOYSTICK_MAX) recoil_val = GAMEPAD_JOYSTICK_MAX;
                    _controller_host_state.ry = recoil_val;
                }

                if (controller_report.buttonL1) {
                    if (getMillis() - turbo_timer > 40) {
                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }
                    if (turbo_state) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                } else {
                    turbo_state = false;
                }

                if (controller_report.buttonSelect && !controller_report.buttonStart) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                }

                if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            // --- COMUNES ---
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

// --------------------------------------------------------------------------------
//                          UPDATE LEDs PS5 (DualSense)
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::update_ds5() {
    if (!ds5_led_needs_update) return;

    DS5OutReport out_report;
    memset(&out_report, 0, sizeof(DS5OutReport));

    out_report.report_id = 0x02;
    out_report.control_flag[0] = 0x02;
    out_report.control_flag[1] = 0x02;
    out_report.led_control_flag = 0x02 | 0x04; // Lightbar + Player LEDs
    out_report.pulse_option = 0x01;
    out_report.led_brightness = 0x00;   // Brillo máximo
    out_report.player_number = 0x04;    // LED central del player indicator

    if (current_profile == PROFILE_EAFC) {
        out_report.lightbar_red   = LED_EAFC_R;
        out_report.lightbar_green = LED_EAFC_G;
        out_report.lightbar_blue  = LED_EAFC_B;   // AZUL
    } else {
        out_report.lightbar_red   = LED_WARZONE_R;
        out_report.lightbar_green = LED_WARZONE_G;
        out_report.lightbar_blue  = LED_WARZONE_B; // ROJO
    }

    if (tuh_hid_send_report(_controller_dev_addr, _controller_instance, 0, &out_report, sizeof(DS5OutReport))) {
        ds5_led_needs_update = false; // Solo marcar como hecho si el envío fue exitoso
    }
}

// --------------------------------------------------------------------------------
//                                  UPDATE CONTROLLER
// --------------------------------------------------------------------------------

void GamepadUSBHostListener::update_ctrlr() {
    if (controller_pid == DS4_ORG_PRODUCT_ID || controller_pid == DS4_PRODUCT_ID ||
        controller_pid == PS4_PRODUCT_ID || controller_pid == PS4_WHEEL_PRODUCT_ID ||
        controller_pid == 0xB67B || controller_pid == 0x00EE) {
        
        if (isDS4Identified) update_ds4();
    }
    // --- NUEVO: Actualizar LED del DualSense PS5 ---
    else if (controller_pid == 0x0CE6) {
        update_ds5();
    }
}

void GamepadUSBHostListener::update_ds4() {
#if GAMEPAD_HOST_USE_FEATURES
    // FUNCIÓN VACÍA - SIN VIBRACIÓN NI LEDS PARA EVITAR ERRORES
#endif
}

// Helpers
bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_get_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}
bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_set_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}
void GamepadUSBHostListener::set_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {
    awaiting_cb = false;
}
void GamepadUSBHostListener::get_report_complete(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len) {
    if (!isDS4Identified) {
        if (report_id == PS4AuthReport::PS4_DEFINITION) setup_ds4();
    }
    awaiting_cb = false;
}
uint32_t GamepadUSBHostListener::map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
bool GamepadUSBHostListener::diff_than_2(uint8_t x, uint8_t y) {
    return (x - y > 2) || (y - x > 2);
}
bool GamepadUSBHostListener::diff_report(PS4Report const* rpt1, PS4Report const* rpt2) {
    bool result;
    result = diff_than_2(rpt1->leftStickX, rpt2->leftStickX) || diff_than_2(rpt1->leftStickY, rpt2->leftStickY) ||
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

// Funciones vacías
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
