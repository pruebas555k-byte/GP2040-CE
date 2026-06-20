#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"
#include <string.h>
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

#define ANTI_RECOIL_STRENGTH 3500

#define LED_DEFAULT_R 0x00
#define LED_DEFAULT_G 0x00
#define LED_DEFAULT_B 0xFF

#define DUALSENSE_PRODUCT_ID 0x0CE6
#define DUALSENSE_EDGE_PRODUCT_ID 0x0DF2

enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile = PROFILE_EAFC;
static bool profile_switch_held = false;

static uint32_t last_hid_report_time = 0;

static bool macro_mute_active = false;
static uint32_t macro_mute_start_time = 0;

static uint32_t turbo_timer = 0;
static bool turbo_state = false;


// Mapeo lineal puro 1:1
static uint16_t applyLinear(uint8_t raw) {
    return (uint16_t)((uint32_t)raw * (GAMEPAD_JOYSTICK_MAX - GAMEPAD_JOYSTICK_MIN) / 255 + GAMEPAD_JOYSTICK_MIN);
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

    // Watchdog suave:
    // Si por alguna razon la lectura HID no quedo armada, se reintenta.
    // Si ya habia una lectura pendiente, TinyUSB devuelve false y no afecta.
    if (_controller_host_enabled && (getMillis() - last_hid_report_time > 1000)) {
        tuh_hid_receive_report(_controller_dev_addr, _controller_instance);
        last_hid_report_time = getMillis();
    }

    if (_controller_host_enabled && getMillis() > _next_update) {
        update_ctrlr();
        _next_update = getMillis() + GAMEPAD_HOST_POLL_INTERVAL_MS;
    }
}

void GamepadUSBHostListener::mount(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    (void)desc_report;
    (void)desc_len;

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

    isDS4Identified = false;
    hasDS4DefReport = false;

    // Como el primer codigo: cada montaje/reconexion arranca en EAFC.
    current_profile = PROFILE_EAFC;
    profile_switch_held = false;

    macro_mute_active = false;
    macro_mute_start_time = 0;

    turbo_timer = 0;
    turbo_state = false;


    last_hid_report_time = getMillis();

    // DualSense necesita init/output report para empezar bien.
    // DualShock no necesita get_report aqui; solo lectura HID.
    if (controller_pid == DUALSENSE_PRODUCT_ID || controller_pid == DUALSENSE_EDGE_PRODUCT_ID) {
        init_ds5_led(dev_addr, instance);
    } else {
        tuh_hid_receive_report(dev_addr, instance);
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


    last_hid_report_time = 0;
}

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    // DualSense necesita este output report para empezar a mandar inputs por USB.
    // Se mantiene la ruta del codigo que prendia el DualSense, pero sin while infinito.
    uint8_t buf[47];

    memset(buf, 0, sizeof(buf));

    buf[1] = 0x14;
    buf[38] = 0x02;
    buf[41] = 0x01;
    buf[42] = 0x02;
    buf[43] = 0x04;

    buf[44] = LED_DEFAULT_R;
    buf[45] = LED_DEFAULT_G;
    buf[46] = LED_DEFAULT_B;

    bool sent = tuh_hid_send_report(dev_addr, instance, 0x02, buf, 47);

    if (!sent) {
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

        // Reintentar un poco, pero nunca infinito.
        uint32_t start = getMillis();
        while (getMillis() - start < 120) {
            tuh_task();

            if (tuh_hid_send_report(dev_addr, instance, 0, buf2, 48)) {
                sent = true;
                break;
            }
        }
    }

    // Arrancar lectura. Si el envio aun esta ocupado, set_report_complete()
    // tambien volvera a armar lectura cuando termine.
    tuh_hid_receive_report(dev_addr, instance);
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if (_controller_host_enabled == false) return;
    if (dev_addr != _controller_dev_addr) return;
    if (instance != _controller_instance) return;

    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    if (itf_protocol != HID_ITF_PROTOCOL_KEYBOARD) {
        last_hid_report_time = getMillis();
        process_ctrlr_report(dev_addr, report, len);
    }

    // Esto es CLAVE para DualSense:
    // despues de cada reporte hay que pedir el siguiente.
    tuh_hid_receive_report(dev_addr, instance);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    (void)dev_addr;

    if (report == nullptr || len < 1) return;

    switch(controller_pid) {
        case DS4_ORG_PRODUCT_ID:
        case DS4_PRODUCT_ID:
        case PS4_WHEEL_PRODUCT_ID:
        case 0xB67B:
        case 0x00EE:
            process_ds4(report, len);
            break;

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

    {
        // Todos los analogos lineales 1:1 en ambos perfiles.
        // Cuadrado no modifica ningun stick.
        _controller_host_state.lx = applyLinear(controller_report.leftStickX);
        _controller_host_state.ly = applyLinear(controller_report.leftStickY);
        _controller_host_state.rx = applyLinear(controller_report.rightStickX);
        _controller_host_state.ry = applyLinear(controller_report.rightStickY);

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
                _controller_host_state.buttons |= GAMEPAD_MASK_B3;
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
    if (report == nullptr || len < 8) return;

    // Parser manual DualSense.
    // TinyUSB puede entregar el reporte con ID 0x01 incluido o sin ese byte.
    // Aqui ajustamos base sin copiar bytes corridos a DSReport.
    uint8_t base = 0;

    if (report[0] == 0x01) {
        if (len < 10) return;
        base = 1;
    } else {
        if (len < 9) return;
        base = 0;
    }

    uint8_t leftStickX   = report[base + 0];
    uint8_t leftStickY   = report[base + 1];
    uint8_t rightStickX  = report[base + 2];
    uint8_t rightStickY  = report[base + 3];
    uint8_t leftTrigger  = report[base + 4];
    uint8_t rightTrigger = report[base + 5];

    uint8_t b0 = report[base + 7];
    uint8_t b1 = report[base + 8];
    uint8_t b2 = report[base + 9];

    uint8_t dpad = b0 & 0x0F;

    bool buttonWest     = (b0 & 0x10) != 0; // Square
    bool buttonSouth    = (b0 & 0x20) != 0; // Cross
    bool buttonEast     = (b0 & 0x40) != 0; // Circle
    bool buttonNorth    = (b0 & 0x80) != 0; // Triangle

    bool buttonL1       = (b1 & 0x01) != 0;
    bool buttonR1       = (b1 & 0x02) != 0;
    bool buttonSelect   = (b1 & 0x10) != 0; // Create
    bool buttonStart    = (b1 & 0x20) != 0; // Options
    bool buttonL3       = (b1 & 0x40) != 0;
    bool buttonR3       = (b1 & 0x80) != 0;

    bool buttonHome     = (b2 & 0x01) != 0;
    bool buttonTouchpad = (b2 & 0x02) != 0;

    {
        // Todos los analogos lineales 1:1 en ambos perfiles.
        // Cuadrado no modifica ningun stick.
        _controller_host_state.lx = applyLinear(leftStickX);
        _controller_host_state.ly = applyLinear(leftStickY);
        _controller_host_state.rx = applyLinear(rightStickX);
        _controller_host_state.ry = applyLinear(rightStickY);

        _controller_host_state.lt = 0;
        _controller_host_state.rt = 0;
        _controller_host_state.buttons = 0;

        _controller_host_analog = true;

        if (buttonSelect && buttonStart) {
            if (!profile_switch_held) {
                profile_switch_held = true;
                current_profile = (current_profile == PROFILE_EAFC) ? PROFILE_WARZONE : PROFILE_EAFC;
            }
        } else {
            profile_switch_held = false;
        }

        if (current_profile == PROFILE_EAFC) {
            if (buttonHome && !macro_mute_active) {
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

            if (buttonWest) {
                _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            }

            if (buttonR1) {
                _controller_host_state.rt = 255;
            }

            if (buttonL1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            _controller_host_state.lt = rightTrigger;

            if (leftTrigger > 160) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (buttonSelect) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S1;
            }

            if (buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }
        } else {
            if (rightTrigger > 200 && leftTrigger > 200) {
                uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;

                if (recoil_val > GAMEPAD_JOYSTICK_MAX) {
                    recoil_val = GAMEPAD_JOYSTICK_MAX;
                }

                _controller_host_state.ry = recoil_val;
            }

            if (buttonL1) {
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

            if (buttonWest) {
                _controller_host_state.buttons |= GAMEPAD_MASK_B3;
            }

            if (buttonSelect && !buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_L1;
            }

            if (buttonR1) {
                _controller_host_state.buttons |= GAMEPAD_MASK_R1;
            }

            if (buttonStart) {
                _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            if (buttonHome) {
                _controller_host_state.buttons |= GAMEPAD_MASK_A1;
            }

            _controller_host_state.lt = leftTrigger;
            _controller_host_state.rt = rightTrigger;
        }

        if (buttonL3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_L3;
        }

        if (buttonR3) {
            _controller_host_state.buttons |= GAMEPAD_MASK_R3;
        }

        if (buttonTouchpad) {
            _controller_host_state.buttons |= GAMEPAD_MASK_A2;
        }

        _controller_host_state.dpad = 0;

        if (dpad == PS4_HAT_UP) {
            _controller_host_state.dpad |= GAMEPAD_MASK_UP;
        }

        if (dpad == PS4_HAT_RIGHT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
        }

        if (dpad == PS4_HAT_DOWN) {
            _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
        }

        if (dpad == PS4_HAT_LEFT) {
            _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;
        }

        if (buttonNorth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B4;
        }

        if (buttonEast) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B2;
        }

        if (buttonSouth) {
            _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        }
    }
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
    isDS4Identified = true;
    hasDS4DefReport = false;
}

void GamepadUSBHostListener::init_ds4(const uint8_t* descReport, uint16_t descLen) {
    (void)descReport;
    (void)descLen;

    // No pedir feature reports DS4 en mount para evitar microcortes.
    isDS4Identified = true;
    hasDS4DefReport = false;
}
