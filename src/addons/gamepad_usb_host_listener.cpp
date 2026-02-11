#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"

// Definición de controladores soportados (Solo PlayStation)
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

// --- VARIABLES GLOBALES PARA PERFILES ---
enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile = PROFILE_EAFC; // Empieza en EA FC
static uint32_t profile_switch_timer = 0;
static bool profile_switch_held = false;

// Variables para Macro Mute (EA FC)
static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;

// Variables para Macro Turbo X (Warzone)
static uint32_t turbo_timer = 0;
static bool turbo_state = false;

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
    gamepad->state.dpad     |= _controller_host_state.dpad;
    gamepad->state.buttons  |= _controller_host_state.buttons;
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
    uint16_t vid = 0;
    uint16_t pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    _controller_host_enabled = true;
    _controller_dev_addr = dev_addr;
    _controller_instance = instance;
    controller_vid = vid;
    controller_pid = pid;

#if GAMEPAD_HOST_DEBUG
    printf("Mount: VID_%04x PID_%04x\n", controller_vid, controller_pid);
#endif

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;
    _controller_host_state.lx = joystick_mid;
    _controller_host_state.ly = joystick_mid;
    _controller_host_state.rx = joystick_mid;
    _controller_host_state.ry = joystick_mid;

    // Solo inicializamos si es PS4/PS5
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
        default:
            // Otros dispositivos (PS5 por defecto no necesita init especial aquí si usas process_ds)
            break;
    }
}

void GamepadUSBHostListener::xmount(uint8_t dev_addr, uint8_t instance, uint8_t controllerType, uint8_t subtype) {
    // XInput eliminado
}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    controller_pid = 0x00;
    controller_vid = 0x00;
    _controller_dev_addr = 0;
    _controller_instance = 0;
    isDS4Identified = false;
    hasDS4DefReport = false;
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if ( _controller_host_enabled == false ) return;

    // Solo procesamos reportes de HID (PS4/PS5)
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
            if (isDS4Identified) {
                process_ds4(report, len);
            }
            break;
        case 0x0CE6: // DualSense (PS5)
            process_ds(report, len);
            break;
        default:
            // Intentar procesar como PS4 genérico si no coincide ID
            // process_ds4(report, len); 
            break;
    }
}

void GamepadUSBHostListener::update_ctrlr() {
    switch(controller_pid)
    {
        case DS4_ORG_PRODUCT_ID:
        case DS4_PRODUCT_ID:    
        case PS4_PRODUCT_ID:    
        case PS4_WHEEL_PRODUCT_ID: 
        case 0xB67B:            
        case 0x00EE:            
            if (isDS4Identified) {
                update_ds4();
            }
            break;
        default:
            break;
    }
}

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
        switch (report_id) {
            case PS4AuthReport::PS4_DEFINITION:
                setup_ds4();
                break;
            default:
                break;
        }
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
    if (hasDS4DefReport) {
        memcpy(&ds4Config, report_buffer+1, sizeof(PS4ControllerConfig));
    }
    if ((ds4Config.hidUsage == 0x2721) || (ds4Config.hidUsage == 0x2127)) {
        isDS4Identified = true;
    }
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

void GamepadUSBHostListener::update_ds4() {
#if GAMEPAD_HOST_USE_FEATURES
    Gamepad * gamepad = Storage::getInstance().GetProcessedGamepad();
    PS4FeatureOutputReport controller_output;
    memset(&controller_output, 0, sizeof(controller_output));
    controller_output.reportID = PS4AuthReport::PS4_SET_FEATURE_STATE;

    if (ds4Config.features.enableLED && gamepad->auxState.sensors.statusLight.enabled) {
        controller_output.enableUpdateLED = gamepad->auxState.sensors.statusLight.enabled;
        controller_output.ledRed = gamepad->auxState.sensors.statusLight.color.red;
        controller_output.ledGreen = gamepad->auxState.sensors.statusLight.color.green;
        controller_output.ledBlue = gamepad->auxState.sensors.statusLight.color.blue;
        controller_output.ledBlinkOn = gamepad->auxState.playerID.ledBlinkOn;
        controller_output.ledBlinkOff = gamepad->auxState.playerID.ledBlinkOff;
    }

    if (ds4Config.features.enableRumble) {
        gamepad->auxState.haptics.leftActuator.enabled = 1;
        gamepad->auxState.haptics.rightActuator.enabled = 1;
        controller_output.enableUpdateRumble = 1;
        controller_output.rumbleLeft = gamepad->auxState.haptics.leftActuator.intensity;
        controller_output.rumbleRight = gamepad->auxState.haptics.rightActuator.intensity;
    }

    void * report = &controller_output;
    uint16_t report_size = sizeof(controller_output)-1;
    tuh_hid_send_report(_controller_dev_addr, _controller_instance, 5, (uint8_t*)report+1, report_size);
#endif
}

// ====================================================================================
//                                  LÓGICA PRINCIPAL
// ====================================================================================

void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    PS4Report controller_report;
    static PS4Report prev_report = { 0 };
    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        // Actualizamos si hay cambios O si hay una macro activa (para el timing)
        if ( diff_report(&prev_report, &controller_report) || macro_mute_active || turbo_state) {
            
            // 1. Sticks Analógicos (Igual para todos)
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            
            // 2. Limpieza Base
            _controller_host_state.lt = 0;
            _controller_host_state.rt = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog = true;

            // ------------------------------------------------------------
            // LÓGICA DE CAMBIO DE PERFIL (Select + Start por 2 segundos)
            // ------------------------------------------------------------
            if (controller_report.buttonSelect && controller_report.buttonStart) {
                if (!profile_switch_held) {
                    profile_switch_held = true;
                    profile_switch_timer = getMillis();
                } else if (getMillis() - profile_switch_timer > 2000) {
                    // Cambiar perfil
                    if (current_profile == PROFILE_EAFC) {
                        current_profile = PROFILE_WARZONE;
                        // Feedback visual opcional si tienes LED
                    } else {
                        current_profile = PROFILE_EAFC;
                    }
                    profile_switch_timer = getMillis() + 5000; // Evitar rebote rápido
                }
            } else {
                profile_switch_held = false;
            }

            // ------------------------------------------------------------
            //                      PERFIL 1: EA FC 26
            // ------------------------------------------------------------
            if (current_profile == PROFILE_EAFC) {
                
                // --- MACRO MUTE (Activador: PS Button) ---
                if (controller_report.buttonHome && !macro_mute_active) {
                    macro_mute_active = true;
                    macro_mute_start_time = getMillis();
                }

                if (macro_mute_active) {
                    if (getMillis() - macro_mute_start_time < 480) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3; // Cuadrado
                        _controller_host_state.buttons |= GAMEPAD_MASK_B2; // Círculo
                    } else {
                        macro_mute_active = false;
                    }
                }

                // --- REMAPEO GATILLOS EA FC ---
                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                
                // R2 -> LT
                if (controller_report.buttonR2 || controller_report.rightTrigger > 10) _controller_host_state.lt = 255;
                // L2 -> R1
                if (controller_report.buttonL2 || controller_report.leftTrigger > 10) _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                // Resto de botones
                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            // ------------------------------------------------------------
            //                      PERFIL 2: WARZONE
            // ------------------------------------------------------------
            else if (current_profile == PROFILE_WARZONE) {
                
                // --- MACRO TURBO X (Activador: L1) ---
                if (controller_report.buttonL1) {
                    // Turbo cada 40ms
                    if (getMillis() - turbo_timer > 40) {
                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }
                    if (turbo_state) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B1; // X / Cruz
                    }
                    // NOTA: NO enviamos L1 (GAMEPAD_MASK_L1) aquí.
                } else {
                    turbo_state = false;
                }

                // --- REMAPEO SELECT -> L1 ---
                // Solo si no estamos cambiando de perfil
                if (controller_report.buttonSelect && !controller_report.buttonStart) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                }

                // --- MAPPING ESTÁNDAR PARA EL RESTO ---
                if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                
                // Gatillos Analógicos Normales
                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
                
                // Botón PS (Home) Normal (Opcional, lo activamos aquí si quieres menú)
                if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            }

            // ------------------------------------------------------------
            // BOTONES COMUNES (Salvo que el perfil los sobrescriba explícitamente)
            // ------------------------------------------------------------
            
            // Sticks Clicks
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            
            // Panel Táctil
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;

            // D-Pad
            if (controller_report.dpad == PS4_HAT_UP) _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_UPRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_UP | GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWNRIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT | GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWN) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_DOWNLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_DOWN | GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_LEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
            if (controller_report.dpad == PS4_HAT_UPLEFT) _controller_host_state.dpad |= GAMEPAD_MASK_LEFT | GAMEPAD_MASK_UP;

            // Botones frontales (Añadimos con OR, para respetar las macros)
            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4; // Triangulo
            if (controller_report.buttonEast) _controller_host_state.buttons |= GAMEPAD_MASK_B2;  // Circulo
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1; // X
            if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;  // Cuadrado
        }
    }
    prev_report = controller_report;
}

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    DSReport controller_report;
    static DSReport prev_ds_report = { 0 };
    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if ( prev_ds_report.reportCounter != controller_report.reportCounter || macro_mute_active || turbo_state) {
            
            // 1. Sticks
            _controller_host_state.lx = map(controller_report.leftStickX, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            
            // 2. Limpieza
            _controller_host_state.lt = 0;
            _controller_host_state.rt = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog = true;

            // ------------------------------------------------------------
            // CAMBIO DE PERFIL (Select + Start)
            // ------------------------------------------------------------
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

            // ------------------------------------------------------------
            // PERFIL 1: EA FC 26 (DualSense)
            // ------------------------------------------------------------
            if (current_profile == PROFILE_EAFC) {
                // Macro Mute
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

                // Remapeo
                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR2 || controller_report.rightTrigger > 10) _controller_host_state.lt = 255;
                if (controller_report.buttonL2 || controller_report.leftTrigger > 10) _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            // ------------------------------------------------------------
            // PERFIL 2: WARZONE (DualSense)
            // ------------------------------------------------------------
            else if (current_profile == PROFILE_WARZONE) {
                // Turbo X en L1
                if (controller_report.buttonL1) {
                    if (getMillis() - turbo_timer > 40) {
                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }
                    if (turbo_state) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                } else {
                    turbo_state = false;
                }

                // Select es L1
                if (controller_report.buttonSelect && !controller_report.buttonStart) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                }

                // Estándar
                if (controller_report.buttonR1) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome) _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            // ------------------------------------------------------------
            // COMUNES
            // ------------------------------------------------------------
            if (controller_report.buttonL3) _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonR3) _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;

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
