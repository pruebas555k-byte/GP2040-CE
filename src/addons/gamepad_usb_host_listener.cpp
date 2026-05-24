#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"
#include <string.h>
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

#define ANTI_RECOIL_STRENGTH 2600

#define LED_DEFAULT_R 0x00
#define LED_DEFAULT_G 0x00
#define LED_DEFAULT_B 0xFF

#define DUALSENSE_PRODUCT_ID 0x0CE6
#define DUALSENSE_EDGE_PRODUCT_ID 0x0DF2

static uint32_t hid_receive_watchdog_timer = 0;

enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile = PROFILE_EAFC;
static bool profile_switch_held = false;

static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;

static uint32_t turbo_timer = 0;
static bool turbo_state = false;

static bool square_hold_active = false;
static bool square_locked = false;
static uint32_t square_hold_start = 0;

#define DEADZONE_RAW 6

// [SQUARE MODE] - activo mientras cuadrado apretado + 300ms despues de soltar
static bool sq_mode_active = false;
static bool sq_was_pressed = false;
static uint32_t sq_release_time = 0;

#define SQ_MODE_TAIL_MS 300

// [SQUARE MODE] - Remeson al presionar cuadrado
static bool sq_bump_active = false;
static uint32_t sq_bump_start = 0;

#define SQ_BUMP_MS 50
#define SQ_BUMP_OFFSET 1800

// Curva izquierdo EAFC sin cuadrado
static uint16_t applyCurve(uint8_t raw) {
    int offset = (int)raw - 128;
    int sign = (offset >= 0) ? 1 : -1;
    int mag = (offset < 0) ? -offset : offset;

    if (mag <= DEADZONE_RAW) {
        return (uint16_t)GAMEPAD_JOYSTICK_MID;
    }

    int mag255 = (mag * 255 + 64) / 128;
    if (mag255 > 255) mag255 = 255;

    int out255;

    if (mag255 <= 26) {
        out255 = 102 * mag255 / 26;
    } else if (mag255 <= 128) {
        out255 = 102 + (179 - 102) * (mag255 - 26) / (128 - 26);
    } else if (mag255 <= 200) {
        out255 = 179 + (228 - 179) * (mag255 - 128) / (200 - 128);
    } else {
        out255 = 228 + (255 - 228) * (mag255 - 200) / (255 - 200);
    }

    int out128 = out255 * 128 / 255;
    int raw_out = 128 + sign * out128;

    if (raw_out < 0) raw_out = 0;
    if (raw_out > 255) raw_out = 255;

    return (uint16_t)((uint32_t)raw_out * (GAMEPAD_JOYSTICK_MAX - GAMEPAD_JOYSTICK_MIN) / 255 + GAMEPAD_JOYSTICK_MIN);
}

// Mapeo lineal puro 1:1
static uint16_t applyLinear(uint8_t raw) {
    return (uint16_t)((uint32_t)raw * (GAMEPAD_JOYSTICK_MAX - GAMEPAD_JOYSTICK_MIN) / 255 + GAMEPAD_JOYSTICK_MIN);
}

// [SQUARE MODE] - Remeson solo en eje X al presionar cuadrado
static void applySquareBump(uint16_t &lx, uint16_t &ly) {
    if (!sq_bump_active) return;

    (void)ly;

    int32_t x = (int32_t)lx - SQ_BUMP_OFFSET;

    if (x < (int32_t)GAMEPAD_JOYSTICK_MIN) x = GAMEPAD_JOYSTICK_MIN;
    if (x > (int32_t)GAMEPAD_JOYSTICK_MAX) x = GAMEPAD_JOYSTICK_MAX;

    lx = (uint16_t)x;
}

// [SQUARE MODE] - Actualizar estado segun si cuadrado esta apretado
static void updateSquareMode(bool buttonWest) {
    if (buttonWest) {
        if (!sq_was_pressed) {
            sq_bump_active = true;
            sq_bump_start = getMillis();
        }

        sq_mode_active = true;
        sq_was_pressed = true;
        sq_release_time = 0;
    } else {
        if (sq_was_pressed) {
            sq_was_pressed = false;
            sq_release_time = getMillis();
        }

        if (sq_mode_active && sq_release_time != 0) {
            if (getMillis() - sq_release_time >= SQ_MODE_TAIL_MS) {
                sq_mode_active = false;
            }
        }
    }

    if (sq_bump_active && getMillis() - sq_bump_start >= SQ_BUMP_MS) {
        sq_bump_active = false;
    }
}

void GamepadUSBHostListener::setup() {
    _controller_host_enabled = false;

    // Perfil inicial al prender firmware.
    current_profile = PROFILE_EAFC;
    profile_switch_held = false;

#if GAMEPAD_HOST_DEBUG
    stdio_init_all();
#endif
}

void GamepadUSBHostListener::process() {
    Gamepad *gamepad = Storage::getInstance().GetGamepad();

    gamepad->hasAnalogTriggers = _controller_host_analog;
    gamepad->hasLeftAnalogStick = _controller_host_analog;
    gamepad->hasRightAnalogStick = _controller_host_analog;

    gamepad->state.dpad = _controller_host_state.dpad;
    gamepad->state.buttons = _controller_host_state.buttons;

    gamepad->state.lx = _controller_host_state.lx;
    gamepad->state.ly = _controller_host_state.ly;
    gamepad->state.rx = _controller_host_state.rx;
    gamepad->state.ry = _controller_host_state.ry;

    gamepad->state.rt = _controller_host_state.rt;
    gamepad->state.lt = _controller_host_state.lt;

    // Watchdog suave para HID host:
    // Si por cualquier razon no quedo armada la lectura, la reintentamos.
    // Si ya hay una lectura pendiente, TinyUSB simplemente devuelve false y no pasa nada.
    if (_controller_host_enabled && getMillis() - hid_receive_watchdog_timer > 100) {
        tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
        hid_receive_watchdog_timer = getMillis();
    }

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
    _controller_type = 0;

    controller_vid = vid;
    controller_pid = pid;

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;

    _controller_host_state.lx = joystick_mid;
    _controller_host_state.ly = joystick_mid;
    _controller_host_state.rx = joystick_mid;
    _controller_host_state.ry = joystick_mid;

    _controller_host_state.lt = 0;
    _controller_host_state.rt = 0;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;

    _controller_host_analog = true;

    isDS4Identified = true;
    hasDS4DefReport = false;

    // El usuario quiere que cada reinicio/reconexion arranque en EAFC.
    current_profile = PROFILE_EAFC;
    profile_switch_held = false;

    macro_mute_active = false;
    macro_mute_start_time = 0;

    turbo_timer = 0;
    turbo_state = false;

    square_hold_active = false;
    square_locked = false;
    square_hold_start = 0;

    sq_mode_active = false;
    sq_was_pressed = false;
    sq_release_time = 0;
    sq_bump_active = false;
    sq_bump_start = 0;

    hid_receive_watchdog_timer = 0;

    switch(controller_pid) {
        // DualShock 4 v2 / Sony 0x09CC.
        // PS4_PRODUCT_ID y DS4_PRODUCT_ID son el mismo valor, por eso solo se usa uno aqui.
        case PS4_PRODUCT_ID:
        case PS4_WHEEL_PRODUCT_ID:
        case 0xB67B:
        case 0x00EE:
            init_ds4(desc_report, desc_len);
            isDS4Identified = true;
            tuh_hid_receive_report(dev_addr, instance);
            break;

        // DualShock 4 original 0x05C4.
        case DS4_ORG_PRODUCT_ID:
            isDS4Identified = true;
            setup_ds4();
            tuh_hid_receive_report(dev_addr, instance);
            break;

        // DualSense 0x0CE6 / DualSense Edge 0x0DF2.
        // Mantener la ruta que encendia y hacia funcionar el DualSense:
        // output report + lectura HID.
        case DUALSENSE_PRODUCT_ID:
        case DUALSENSE_EDGE_PRODUCT_ID:
            init_ds5_led(dev_addr, instance);
            break;

        default:
            tuh_hid_receive_report(dev_addr, instance);
            break;
    }
}

void GamepadUSBHostListener::xmount(uint8_t dev_addr, uint8_t instance, uint8_t controllerType, uint8_t subtype) {
    (void)dev_addr;
    (void)instance;
    (void)controllerType;
    (void)subtype;
}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    (void)dev_addr;

    _controller_host_enabled = false;

    controller_pid = 0x00;
    controller_vid = 0x00;

    _controller_dev_addr = 0;
    _controller_instance = 0;
    _controller_type = 0;

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;

    _controller_host_state.lx = joystick_mid;
    _controller_host_state.ly = joystick_mid;
    _controller_host_state.rx = joystick_mid;
    _controller_host_state.ry = joystick_mid;

    _controller_host_state.lt = 0;
    _controller_host_state.rt = 0;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad = 0;

    isDS4Identified = false;
    hasDS4DefReport = false;

    macro_mute_active = false;
    macro_mute_start_time = 0;

    turbo_timer = 0;
    turbo_state = false;

    square_hold_active = false;
    square_locked = false;
    square_hold_start = 0;

    sq_mode_active = false;
    sq_was_pressed = false;
    sq_release_time = 0;
    sq_bump_active = false;
    sq_bump_start = 0;

    hid_receive_watchdog_timer = 0;
}

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    // DualSense necesita este output report para empezar bien por USB.
    // Importante: no usar while infinito.
    uint8_t buf[47];
    bool send_ok = false;

    memset(buf, 0, sizeof(buf));

    buf[1] = 0x14;
    buf[38] = 0x02;
    buf[41] = 0x01;
    buf[42] = 0x02;
    buf[43] = 0x04;

    buf[44] = LED_DEFAULT_R;
    buf[45] = LED_DEFAULT_G;
    buf[46] = LED_DEFAULT_B;

    send_ok = tuh_hid_send_report(dev_addr, instance, 0x02, buf, 47);

    if (!send_ok) {
        uint8_t buf2[48];

        memset(buf2, 0, sizeof(buf2));

        buf2[0] = 0x02;
        buf2[2] = 0x14;
        buf2[39] = 0x02;
        buf2[42] = 0x01;
        buf2[43] = 0x02;
        buf2[44] = 0x04;

        buf2[45] = LED_DEFAULT_R;
        buf2[46] = LED_DEFAULT_G;
        buf2[47] = LED_DEFAULT_B;

        uint32_t start = getMillis();

        while (getMillis() - start < 500) {
            tuh_task();

            if (tuh_hid_send_report(dev_addr, instance, 0, buf2, 48)) {
                send_ok = true;
                break;
            }
        }
    }

    // Si no se pudo mandar el output report, igual intentamos leer.
    // Si si se pudo mandar, set_report_complete() tambien armara la lectura
    // cuando termine el envio, que es mas seguro para DualSense.
    if (!send_ok) {
        tuh_hid_receive_report(dev_addr, instance);
    }
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if (_controller_host_enabled == false) return;
    if (dev_addr != _controller_dev_addr) return;
    if (instance != _controller_instance) return;

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if (itf_protocol != HID_ITF_PROTOCOL_KEYBOARD) {
        process_ctrlr_report(dev_addr, report, len);
    }

    // Re-armar lectura despues de cada reporte.
    // Esto evita que DualSense encienda luz pero el RP no reciba botones.
    tuh_hid_receive_report(dev_addr, instance);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    (void)dev_addr;

    if (report == nullptr || len < 1) return;

    switch(controller_pid) {
        // DualShock 4 original 0x05C4
        case DS4_ORG_PRODUCT_ID:

        // DualShock 4 v2 / Sony 0x09CC
        // DS4_PRODUCT_ID representa 0x09CC.
        case DS4_PRODUCT_ID:

        // Compatibles PS4 / Wheels
        case PS4_WHEEL_PRODUCT_ID:
        case 0xB67B:
        case 0x00EE:
            process_ds4(report, len);
            break;

        // DualSense 0x0CE6 / DualSense Edge 0x0DF2
        case DUALSENSE_PRODUCT_ID:
        case DUALSENSE_EDGE_PRODUCT_ID:
            process_ds(report, len);
            break;

        default:
            process_ds(report, len);
            break;
    }
}

void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    if (report == nullptr || len < 1) return;

    uint8_t const report_id = report[0];

    if (report_id != 1) return;
    if (len < sizeof(PS4Report)) return;

    PS4Report controller_report;
    static PS4Report prev_report = { 0 };

    memset(&controller_report, 0, sizeof(controller_report));
    memcpy(&controller_report, report, sizeof(controller_report));

    if (diff_report(&prev_report, &controller_report) || macro_mute_active || turbo_state || controller_report.buttonWest || sq_mode_active) {
        if (current_profile == PROFILE_EAFC) {
            updateSquareMode(controller_report.buttonWest);

            // Izquierdo: curva normal, lineal al apretar cuadrado
            if (sq_mode_active) {
                _controller_host_state.lx = applyLinear(controller_report.leftStickX);
                _controller_host_state.ly = applyLinear(controller_report.leftStickY);
                applySquareBump(_controller_host_state.lx, _controller_host_state.ly);
            } else {
                _controller_host_state.lx = applyCurve(controller_report.leftStickX);
                _controller_host_state.ly = applyCurve(controller_report.leftStickY);
            }

            // Derecho: siempre lineal
            _controller_host_state.rx = applyLinear(controller_report.rightStickX);
            _controller_host_state.ry = applyLinear(controller_report.rightStickY);
        } else {
            _controller_host_state.lx = map(controller_report.leftStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
        }

        _controller_host_state.lt = 0;
        _controller_host_state.rt = 0;
        _controller_host_state.buttons = 0;

        _controller_host_analog = true;

        if (controller_report.buttonSelect && controller_report.buttonStart) {
            if (!profile_switch_held) {
                profile_switch_held = true;
                current_profile = (current_profile == PROFILE_EAFC) ? PROFILE_WARZONE : PROFILE_EAFC;
            }
        } else {
            profile_switch_held = false;
        }

        if (current_profile == PROFILE_EAFC) {
            if (controller_report.buttonHome && !macro_mute_active) {
                macro_mute_active = true;
                macro_mute_start_time = getMillis();
            }

            if (macro_mute_active) {
                if (getMillis() - macro_mute_start_time < 484) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                    _controller_host_state.buttons |= GAMEPAD_MASK_B2;
                } else {
                    macro_mute_active = false;
                }
            }

            if (controller_report.buttonWest) {
                if (!square_locked && !square_hold_active) {
                    square_hold_active = true;
                    square_hold_start = getMillis();
                }

                if (square_hold_active) {
                    if (getMillis() - square_hold_start < 4700) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                    } else {
                        square_hold_active = false;
                        square_locked = true;
                    }
                }
            } else {
                square_hold_active = false;
                square_locked = false;
            }

            if (controller_report.buttonR1) {
                _controller_host_state.rt = 255;
            }

            if (controller_report.buttonL1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            _controller_host_state.lt = controller_report.rightTrigger;

            if (controller_report.leftTrigger > 160) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (controller_report.buttonSelect) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            }

            if (controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }
        } else {
            if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
                uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;

                if (recoil_val > GAMEPAD_JOYSTICK_MAX) {
                    recoil_val = GAMEPAD_JOYSTICK_MAX;
                }

                _controller_host_state.ry = recoil_val;
            }

            if (controller_report.buttonL1) {
                if (getMillis() - turbo_timer > 40) {
                    turbo_state = !turbo_state;
                    turbo_timer = getMillis();
                }

                if (turbo_state) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                }
            } else {
                turbo_state = false;
            }

            if (controller_report.buttonWest) {
                _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            }

            if (controller_report.buttonSelect && !controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            if (controller_report.buttonR1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            if (controller_report.buttonHome) {
                _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            }

            _controller_host_state.lt = controller_report.leftTrigger;
            _controller_host_state.rt = controller_report.rightTrigger;
        }

        if (controller_report.buttonL3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_L3;
        }

        if (controller_report.buttonR3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_R3;
        }

        if (controller_report.buttonTouchpad) {
            _controller_host_state.buttons |= GAMEPAD_MASK_A2;
        }

        _controller_host_state.dpad = 0;

        if (controller_report.dpad == PS4_HAT_UP) {
            _controller_host_state.dpad |= GAMEPAD_MASK_UP;
        }

        if (controller_report.dpad == PS4_HAT_RIGHT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
        }

        if (controller_report.dpad == PS4_HAT_DOWN) {
            _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
        }

        if (controller_report.dpad == PS4_HAT_LEFT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
        }

        if (controller_report.buttonNorth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B4;
        }

        if (controller_report.buttonEast) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B2;
        }

        if (controller_report.buttonSouth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        }
    }

    prev_report = controller_report;
}

void GamepadUSBHostListener::process_ds(uint8_t const* report, uint16_t len) {
    if (report == nullptr || len < 1) return;

    uint8_t const report_id = report[0];

    if (report_id != 1) return;
    if (len < sizeof(DSReport)) return;

    DSReport controller_report;
    static DSReport prev_ds_report = { 0 };

    memset(&controller_report, 0, sizeof(controller_report));
    memcpy(&controller_report, report, sizeof(controller_report));

    if (prev_ds_report.reportCounter != controller_report.reportCounter || macro_mute_active || turbo_state || controller_report.buttonWest || sq_mode_active) {
        if (current_profile == PROFILE_EAFC) {
            updateSquareMode(controller_report.buttonWest);

            // Izquierdo: curva normal, lineal al apretar cuadrado
            if (sq_mode_active) {
                _controller_host_state.lx = applyLinear(controller_report.leftStickX);
                _controller_host_state.ly = applyLinear(controller_report.leftStickY);
                applySquareBump(_controller_host_state.lx, _controller_host_state.ly);
            } else {
                _controller_host_state.lx = applyCurve(controller_report.leftStickX);
                _controller_host_state.ly = applyCurve(controller_report.leftStickY);
            }

            // Derecho: siempre lineal
            _controller_host_state.rx = applyLinear(controller_report.rightStickX);
            _controller_host_state.ry = applyLinear(controller_report.rightStickY);
        } else {
            _controller_host_state.lx = map(controller_report.leftStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
        }

        _controller_host_state.lt = 0;
        _controller_host_state.rt = 0;
        _controller_host_state.buttons = 0;

        _controller_host_analog = true;

        if (controller_report.buttonSelect && controller_report.buttonStart) {
            if (!profile_switch_held) {
                profile_switch_held = true;
                current_profile = (current_profile == PROFILE_EAFC) ? PROFILE_WARZONE : PROFILE_EAFC;
            }
        } else {
            profile_switch_held = false;
        }

        if (current_profile == PROFILE_EAFC) {
            if (controller_report.buttonHome && !macro_mute_active) {
                macro_mute_active = true;
                macro_mute_start_time = getMillis();
            }

            if (macro_mute_active) {
                if (getMillis() - macro_mute_start_time < 488) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                    _controller_host_state.buttons |= GAMEPAD_MASK_B2;
                } else {
                    macro_mute_active = false;
                }
            }

            if (controller_report.buttonWest) {
                if (!square_locked && !square_hold_active) {
                    square_hold_active = true;
                    square_hold_start = getMillis();
                }

                if (square_hold_active) {
                    if (getMillis() - square_hold_start < 248) {
                        _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                    } else {
                        square_hold_active = false;
                        square_locked = true;
                    }
                }
            } else {
                square_hold_active = false;
                square_locked = false;
            }

            if (controller_report.buttonR1) {
                _controller_host_state.rt = 255;
            }

            if (controller_report.buttonL1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            _controller_host_state.lt = controller_report.rightTrigger;

            if (controller_report.leftTrigger > 160) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (controller_report.buttonSelect) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            }

            if (controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }
        } else {
            if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {
                uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;

                if (recoil_val > GAMEPAD_JOYSTICK_MAX) {
                    recoil_val = GAMEPAD_JOYSTICK_MAX;
                }

                _controller_host_state.ry = recoil_val;
            }

            if (controller_report.buttonL1) {
                if (getMillis() - turbo_timer > 40) {
                    turbo_state = !turbo_state;
                    turbo_timer = getMillis();
                }

                if (turbo_state) {
                    _controller_host_state.buttons |= GAMEPAD_MASK_B1;
                }
            } else {
                turbo_state = false;
            }

            // FIX WARZONE
            if (controller_report.buttonWest) {
                _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            }

            if (controller_report.buttonSelect && !controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            if (controller_report.buttonR1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (controller_report.buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            if (controller_report.buttonHome) {
                _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            }

            _controller_host_state.lt = controller_report.leftTrigger;
            _controller_host_state.rt = controller_report.rightTrigger;
        }

        if (controller_report.buttonL3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_L3;
        }

        if (controller_report.buttonR3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_R3;
        }

        if (controller_report.buttonTouchpad) {
            _controller_host_state.buttons |= GAMEPAD_MASK_A2;
        }

        _controller_host_state.dpad = 0;

        if (controller_report.dpad == PS4_HAT_UP) {
            _controller_host_state.dpad |= GAMEPAD_MASK_UP;
        }

        if (controller_report.dpad == PS4_HAT_RIGHT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
        }

        if (controller_report.dpad == PS4_HAT_DOWN) {
            _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
        }

        if (controller_report.dpad == PS4_HAT_LEFT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
        }

        if (controller_report.buttonNorth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B4;
        }

        if (controller_report.buttonEast) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B2;
        }

        if (controller_report.buttonSouth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        }
    }

    prev_ds_report = controller_report;
}

void GamepadUSBHostListener::update_ctrlr() {
    if (controller_pid == DS4_ORG_PRODUCT_ID ||
        controller_pid == DS4_PRODUCT_ID ||
        controller_pid == PS4_WHEEL_PRODUCT_ID ||
        controller_pid == 0xB67B ||
        controller_pid == 0x00EE) {

        update_ds4();
    }
}

void GamepadUSBHostListener::update_ds4() {
}

bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_get_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}

bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) {
    awaiting_cb = true;
    return tuh_hid_set_report(_controller_dev_addr, _controller_instance, report_id, HID_REPORT_TYPE_FEATURE, report, len);
}

void GamepadUSBHostListener::set_report_complete(uint8_t, uint8_t, uint8_t, uint8_t, uint16_t) {
    awaiting_cb = false;

    // En DualSense, despues del output report, arrancar lectura HID.
    // Esto arregla el caso: prende luz, pero el RP no recibe inputs.
    if (_controller_host_enabled &&
        (controller_pid == DUALSENSE_PRODUCT_ID || controller_pid == DUALSENSE_EDGE_PRODUCT_ID)) {
        tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
    }
}

void GamepadUSBHostListener::get_report_complete(uint8_t, uint8_t, uint8_t report_id, uint8_t, uint16_t) {
    if (report_id == PS4AuthReport::PS4_DEFINITION) {
        setup_ds4();
    }

    awaiting_cb = false;
}

uint32_t GamepadUSBHostListener::map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool GamepadUSBHostListener::diff_than_2(uint8_t x, uint8_t y) {
    return (x > y) ? ((x - y) > 2) : ((y - x) > 2);
}

bool GamepadUSBHostListener::diff_report(PS4Report const* rpt1, PS4Report const* rpt2) {
    bool result;

    result = diff_than_2(rpt1->leftStickX, rpt2->leftStickX) ||
             diff_than_2(rpt1->leftStickY, rpt2->leftStickY) ||
             diff_than_2(rpt1->rightStickX, rpt2->rightStickX) ||
             diff_than_2(rpt1->rightStickY, rpt2->rightStickY);

    result |= memcmp(&rpt1->rightStickY + 1, &rpt2->rightStickY + 1, sizeof(PS4Report) - 6);

    return result;
}

void GamepadUSBHostListener::setup_ds4() {
    // DS4 no debe quedar bloqueado esperando definition report.
    isDS4Identified = true;
    hasDS4DefReport = false;
}

void GamepadUSBHostListener::init_ds4(const uint8_t* descReport, uint16_t descLen) {
    (void)descReport;
    (void)descLen;

    // DS4 estable:
    // No hacer host_get_report durante mount.
    // En algunos DS4/clones ese control transfer causa microcortes o reenumeracion.
    isDS4Identified = true;
    hasDS4DefReport = false;
}
