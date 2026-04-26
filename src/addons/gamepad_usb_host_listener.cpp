#include "addons/gamepad_usb_host_listener.h"
#include "storagemanager.h"
#include "class/hid/hid.h"
#include "class/hid/hid_host.h"
#include "pico/stdlib.h"
#include <cstring>
#include "drivers/ps4/PS4Descriptors.h"
#include "drivers/ps4/PS4Driver.h"

#define ANTI_RECOIL_STRENGTH 2600

#define LED_DEFAULT_R 0x00
#define LED_DEFAULT_G 0x00
#define LED_DEFAULT_B 0xFF

enum Profile {
    PROFILE_EAFC,
    PROFILE_WARZONE
};

static Profile current_profile        = PROFILE_EAFC;
static bool profile_switch_held       = false;
static bool macro_mute_active         = false;
static uint32_t macro_mute_start_time = 0;
static uint32_t turbo_timer           = 0;
static bool turbo_state               = false;
static bool square_hold_active        = false;
static bool square_locked             = false;
static uint32_t square_hold_start     = 0;

// R1 one-shot (EAFC):
// - R1+R2 juntos → manda ambos 150ms, luego R1 se corta sin importar si R2 sigue apretado
// - Para volver a usar R1: soltar R1 fisicamente y apretarlo solo
static bool r1_disabled     = false;
static bool r1r2_triggered  = false;
static uint32_t r1r2_start  = 0;
static bool r1_was_released = true;

#define R1_ONESHOT_MS 150

#define DEADZONE_RAW 6

// [SQUARE MODE] - estado: activo mientras cuadrado apretado + 300ms despues de soltar
static bool     sq_mode_active    = false;  // esta activo el modo restringido
static bool     sq_was_pressed    = false;  // cuadrado estaba apretado el frame anterior
static uint32_t sq_release_time   = 0;      // momento en que se solto cuadrado
#define SQ_MODE_TAIL_MS 300                 // ms que dura el efecto tras soltar cuadrado

// Curva normal EAFC
static uint16_t applyCurve(uint8_t raw) {
    int offset = (int)raw - 128;
    int sign   = (offset >= 0) ? 1 : -1;
    int mag    = (offset < 0) ? -offset : offset;

    if (mag <= DEADZONE_RAW) {
        return (uint16_t)GAMEPAD_JOYSTICK_MID;
    }

    int mag255 = (mag * 255 + 64) / 128;
    if (mag255 > 255) mag255 = 255;

    int out255;
    if (mag255 <= 26)
        out255 = 102 * mag255 / 26;
    else if (mag255 <= 128)
        out255 = 102 + (179 - 102) * (mag255 - 26)  / (128 - 26);
    else if (mag255 <= 200)
        out255 = 179 + (228 - 179) * (mag255 - 128) / (200 - 128);
    else
        out255 = 228 + (255 - 228) * (mag255 - 200) / (255 - 200);

    int out128  = out255 * 128 / 255;
    int raw_out = 128 + sign * out128;
    if (raw_out < 0)   raw_out = 0;
    if (raw_out > 255) raw_out = 255;

    return (uint16_t)((uint32_t)raw_out * (GAMEPAD_JOYSTICK_MAX - GAMEPAD_JOYSTICK_MIN) / 255
                      + GAMEPAD_JOYSTICK_MIN);
}

// [SQUARE MODE] - Curva mas lenta/resistente para el stick izquierdo
static uint16_t applyCurveSlow(uint8_t raw) {
    int offset = (int)raw - 128;
    int sign   = (offset >= 0) ? 1 : -1;
    int mag    = (offset < 0) ? -offset : offset;

    // Deadzone un poco mas grande
    if (mag <= DEADZONE_RAW + 6) {
        return (uint16_t)GAMEPAD_JOYSTICK_MID;
    }

    int mag255 = (mag * 255 + 64) / 128;
    if (mag255 > 255) mag255 = 255;

    // Curva mucho mas lenta: respuesta suave, no llega al maximo total
    int out255;
    if (mag255 <= 50)
        out255 = 35  * mag255 / 50;
    else if (mag255 <= 130)
        out255 = 35  + (100 - 35)  * (mag255 - 50)  / (130 - 50);
    else if (mag255 <= 210)
        out255 = 100 + (165 - 100) * (mag255 - 130) / (210 - 130);
    else
        out255 = 165 + (200 - 165) * (mag255 - 210) / (255 - 210);

    int out128  = out255 * 128 / 255;
    int raw_out = 128 + sign * out128;
    if (raw_out < 0)   raw_out = 0;
    if (raw_out > 255) raw_out = 255;

    return (uint16_t)((uint32_t)raw_out * (GAMEPAD_JOYSTICK_MAX - GAMEPAD_JOYSTICK_MIN) / 255
                      + GAMEPAD_JOYSTICK_MIN);
}

// [SQUARE MODE] - Aplica restriccion de cono +-45 grados desde arriba + bloqueo mitad inferior
// Trabaja sobre los valores ya mapeados al rango joystick (lx, ly)
static void applySquareModeFilter(uint16_t &lx, uint16_t &ly) {
    // Convertir de rango joystick a -128..+127
    int dx = (int)lx - (int)GAMEPAD_JOYSTICK_MID;
    int dy = (int)ly - (int)GAMEPAD_JOYSTICK_MID;

    // dy positivo = abajo (Y aumenta hacia abajo en PS4)
    // Bloquear mitad inferior: si dy > 0 → clavar al centro
    if (dy > 0) {
        lx = GAMEPAD_JOYSTICK_MID;
        ly = GAMEPAD_JOYSTICK_MID;
        return;
    }

    // Estamos en la mitad superior (dy <= 0)
    // Cono +-45 grados desde arriba: tan(45) = 1, condicion: |dx| <= |dy|
    // Si |dx| > |dy| → estamos fuera del cono, comprimir dx al borde del cono
    int absDy = -dy;  // ahora positivo
    int absDx = (dx < 0) ? -dx : dx;

    if (absDy == 0) {
        // Stick exactamente en el centro o eje horizontal → bloquear
        lx = GAMEPAD_JOYSTICK_MID;
        ly = GAMEPAD_JOYSTICK_MID;
        return;
    }

    if (absDx > absDy) {
        // Fuera del cono: comprimir X al borde +-45 grados (|dx| = |dy|)
        int sign_dx = (dx >= 0) ? 1 : -1;
        dx = sign_dx * absDy;
    }

    // Reconstruir valores joystick
    lx = (uint16_t)((int)GAMEPAD_JOYSTICK_MID + dx);
    ly = (uint16_t)((int)GAMEPAD_JOYSTICK_MID + dy);

    // Clamp por seguridad
    if (lx < GAMEPAD_JOYSTICK_MIN) lx = GAMEPAD_JOYSTICK_MIN;
    if (lx > GAMEPAD_JOYSTICK_MAX) lx = GAMEPAD_JOYSTICK_MAX;
    if (ly < GAMEPAD_JOYSTICK_MIN) ly = GAMEPAD_JOYSTICK_MIN;
    if (ly > GAMEPAD_JOYSTICK_MAX) ly = GAMEPAD_JOYSTICK_MAX;
}

// [SQUARE MODE] - Actualizar estado del modo segun si cuadrado esta apretado
// Llamar ANTES de procesar el stick izquierdo
static void updateSquareMode(bool buttonWest) {
    if (buttonWest) {
        sq_mode_active  = true;
        sq_was_pressed  = true;
        sq_release_time = 0;
    } else {
        if (sq_was_pressed) {
            // Acaba de soltarse: arrancar timer tail
            sq_was_pressed  = false;
            sq_release_time = getMillis();
        }
        // Mantener activo durante SQ_MODE_TAIL_MS tras soltar
        if (sq_mode_active && sq_release_time != 0) {
            if (getMillis() - sq_release_time >= SQ_MODE_TAIL_MS) {
                sq_mode_active = false;
            }
        }
    }
}

#define HANDLE_R1_ONESHOT(btnR1, trigR2)                                      \
    do {                                                                      \
        bool _r1 = (btnR1);                                                   \
        bool _r2 = ((trigR2) > 200);                                          \
                                                                              \
        if (!_r1) {                                                           \
            r1_was_released = true;                                           \
            if (!_r2) r1r2_triggered = false;                                 \
        }                                                                     \
                                                                              \
        if (_r1 && _r2 && !r1r2_triggered) {                                  \
            r1r2_triggered  = true;                                           \
            r1r2_start      = getMillis();                                    \
            r1_disabled     = false;                                          \
            r1_was_released = false;                                          \
        }                                                                     \
                                                                              \
        if (r1r2_triggered) {                                                 \
            if (getMillis() - r1r2_start < R1_ONESHOT_MS) {                  \
                _controller_host_state.rt = 255;                              \
            } else {                                                          \
                r1_disabled = true;                                           \
            }                                                                 \
        }                                                                     \
                                                                              \
        if (_r1 && !_r2 && r1_disabled && r1_was_released) {                  \
            r1_disabled     = false;                                          \
            r1_was_released = false;                                          \
            r1r2_triggered  = false;                                          \
        }                                                                     \
                                                                              \
        if (_r1 && !r1_disabled && !r1r2_triggered) {                         \
            _controller_host_state.rt = 255;                                  \
        }                                                                     \
    } while(0)

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

    gamepad->state.dpad    = _controller_host_state.dpad;
    gamepad->state.buttons = _controller_host_state.buttons;
    gamepad->state.lx      = _controller_host_state.lx;
    gamepad->state.ly      = _controller_host_state.ly;
    gamepad->state.rx      = _controller_host_state.rx;
    gamepad->state.ry      = _controller_host_state.ry;
    gamepad->state.rt      = _controller_host_state.rt;
    gamepad->state.lt      = _controller_host_state.lt;

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
    _controller_dev_addr     = dev_addr;
    _controller_instance     = instance;
    _controller_type         = 0;

    controller_vid = vid;
    controller_pid = pid;

    uint16_t joystick_mid = GAMEPAD_JOYSTICK_MID;

    _controller_host_state.lx      = joystick_mid;
    _controller_host_state.ly      = joystick_mid;
    _controller_host_state.rx      = joystick_mid;
    _controller_host_state.ry      = joystick_mid;
    _controller_host_state.buttons = 0;
    _controller_host_state.dpad    = 0;

    isDS4Identified = true;
    current_profile = PROFILE_EAFC;

    init_ds5_led(dev_addr, instance);

    switch(controller_pid) {
        case PS4_PRODUCT_ID:
        case PS4_WHEEL_PRODUCT_ID:
        case 0xB67B:
        case 0x00EE:
            init_ds4(desc_report, desc_len);
            break;
        case DS4_ORG_PRODUCT_ID:
            setup_ds4();
            break;
        case 0x0CE6:
            break;
        default:
            break;
    }
}

void GamepadUSBHostListener::xmount(uint8_t dev_addr, uint8_t instance, uint8_t controllerType, uint8_t subtype) {}

void GamepadUSBHostListener::unmount(uint8_t dev_addr) {
    _controller_host_enabled = false;
    controller_pid           = 0x00;
    controller_vid           = 0x00;
    _controller_dev_addr     = 0;
    _controller_instance     = 0;
    _controller_type         = 0;
    isDS4Identified          = false;
    hasDS4DefReport          = false;
}

void GamepadUSBHostListener::init_ds5_led(uint8_t dev_addr, uint8_t instance) {
    uint8_t buf[47];
    memset(buf, 0, sizeof(buf));

    buf[1]  = 0x14;
    buf[38] = 0x02;
    buf[41] = 0x01;
    buf[42] = 0x02;
    buf[43] = 0x04;
    buf[44] = LED_DEFAULT_R;
    buf[45] = LED_DEFAULT_G;
    buf[46] = LED_DEFAULT_B;

    if (!tuh_hid_send_report(dev_addr, instance, 0x02, buf, 47)) {
        uint8_t buf2[48];
        memset(buf2, 0, sizeof(buf2));
        buf2[0]  = 0x02;
        buf2[2]  = 0x14;
        buf2[39] = 0x02;
        buf2[42] = 0x01;
        buf2[43] = 0x02;
        buf2[44] = 0x04;
        buf2[45] = LED_DEFAULT_R;
        buf2[46] = LED_DEFAULT_G;
        buf2[47] = LED_DEFAULT_B;
        while (!tuh_hid_send_report(dev_addr, instance, 0, buf2, 48)) {
            tuh_task();
        }
    }

    tuh_hid_receive_report(dev_addr, instance);
}

void GamepadUSBHostListener::report_received(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    if (_controller_host_enabled == false) return;
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD) return;
    process_ctrlr_report(dev_addr, report, len);
}

void GamepadUSBHostListener::process_ctrlr_report(uint8_t dev_addr, uint8_t const* report, uint16_t len) {
    switch(controller_pid) {
        case DS4_ORG_PRODUCT_ID:
        case DS4_PRODUCT_ID:
        case PS4_WHEEL_PRODUCT_ID:
        case 0xB67B:
        case 0x00EE:
            if (isDS4Identified) process_ds4(report, len);
            break;
        case 0x0CE6:
            process_ds(report, len);
            break;
        default:
            process_ds(report, len);
            break;
    }
}

void GamepadUSBHostListener::process_ds4(uint8_t const* report, uint16_t len) {
    PS4Report controller_report;
    static PS4Report prev_report = { 0 };

    uint8_t const report_id = report[0];

    if (report_id == 1) {
        memcpy(&controller_report, report, sizeof(controller_report));

        if (diff_report(&prev_report, &controller_report) || macro_mute_active || turbo_state || controller_report.buttonWest || r1r2_triggered || sq_mode_active) {

            if (current_profile == PROFILE_EAFC) {
                // [SQUARE MODE] Actualizar estado del modo antes de calcular el stick
                updateSquareMode(controller_report.buttonWest);

                if (sq_mode_active) {
                    // Curva lenta para el stick izquierdo
                    _controller_host_state.lx = applyCurveSlow(controller_report.leftStickX);
                    _controller_host_state.ly = applyCurveSlow(controller_report.leftStickY);
                    // Aplicar filtro de cono +-45 grados + bloqueo mitad inferior
                    applySquareModeFilter(_controller_host_state.lx, _controller_host_state.ly);
                } else {
                    _controller_host_state.lx = applyCurve(controller_report.leftStickX);
                    _controller_host_state.ly = applyCurve(controller_report.leftStickY);
                }
                _controller_host_state.rx = applyCurve(controller_report.rightStickX);
                _controller_host_state.ry = applyCurve(controller_report.rightStickY);
            } else {
                _controller_host_state.lx = map(controller_report.leftStickX,  0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.ly = map(controller_report.leftStickY,  0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            }

            _controller_host_state.lt      = 0;
            _controller_host_state.rt      = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog        = true;

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
                    macro_mute_active     = true;
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
                        square_hold_start  = getMillis();
                    }
                    if (square_hold_active) {
                        if (getMillis() - square_hold_start < 253) {
                            _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                        } else {
                            square_hold_active = false;
                            square_locked      = true;
                        }
                    }
                } else {
                    square_hold_active = false;
                    square_locked      = false;
                }

                HANDLE_R1_ONESHOT(controller_report.buttonR1, controller_report.rightTrigger);

                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                _controller_host_state.lt = controller_report.rightTrigger;
                if (controller_report.leftTrigger > 160) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart)  _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            } else {
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

                if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                if (controller_report.buttonSelect && !controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR1)   _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome)  _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            if (controller_report.buttonL3)       _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonR3)       _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;

            _controller_host_state.dpad = 0;
            if (controller_report.dpad == PS4_HAT_UP)    _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWN)  _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_LEFT)  _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;

            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast)  _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
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

        if (prev_ds_report.reportCounter != controller_report.reportCounter || macro_mute_active || turbo_state || controller_report.buttonWest || r1r2_triggered || sq_mode_active) {

            if (current_profile == PROFILE_EAFC) {
                // [SQUARE MODE] Actualizar estado del modo antes de calcular el stick
                updateSquareMode(controller_report.buttonWest);

                if (sq_mode_active) {
                    // Curva lenta para el stick izquierdo
                    _controller_host_state.lx = applyCurveSlow(controller_report.leftStickX);
                    _controller_host_state.ly = applyCurveSlow(controller_report.leftStickY);
                    // Aplicar filtro de cono +-45 grados + bloqueo mitad inferior
                    applySquareModeFilter(_controller_host_state.lx, _controller_host_state.ly);
                } else {
                    _controller_host_state.lx = applyCurve(controller_report.leftStickX);
                    _controller_host_state.ly = applyCurve(controller_report.leftStickY);
                }
                _controller_host_state.rx = applyCurve(controller_report.rightStickX);
                _controller_host_state.ry = applyCurve(controller_report.rightStickY);
            } else {
                _controller_host_state.lx = map(controller_report.leftStickX,  0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.ly = map(controller_report.leftStickY,  0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.rx = map(controller_report.rightStickX, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
                _controller_host_state.ry = map(controller_report.rightStickY, 0, 255, GAMEPAD_JOYSTICK_MIN, GAMEPAD_JOYSTICK_MAX);
            }

            _controller_host_state.lt      = 0;
            _controller_host_state.rt      = 0;
            _controller_host_state.buttons = 0;
            _controller_host_analog        = true;

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
                    macro_mute_active     = true;
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
                        square_hold_start  = getMillis();
                    }
                    if (square_hold_active) {
                        if (getMillis() - square_hold_start < 242) {
                            _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                        } else {
                            square_hold_active = false;
                            square_locked      = true;
                        }
                    }
                } else {
                    square_hold_active = false;
                    square_locked      = false;
                }

                HANDLE_R1_ONESHOT(controller_report.buttonR1, controller_report.rightTrigger);

                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                _controller_host_state.lt = controller_report.rightTrigger;
                if (controller_report.leftTrigger > 160) _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonSelect) _controller_host_state.buttons |= GAMEPAD_MASK_S1;
                if (controller_report.buttonStart)  _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            } else {
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

                // FIX WARZONE
                if (controller_report.buttonWest) _controller_host_state.buttons |= GAMEPAD_MASK_B3;
                if (controller_report.buttonSelect && !controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_L1;
                if (controller_report.buttonR1)   _controller_host_state.buttons |= GAMEPAD_MASK_R1;
                if (controller_report.buttonStart) _controller_host_state.buttons |= GAMEPAD_MASK_S2;
                if (controller_report.buttonHome)  _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            if (controller_report.buttonL3)       _controller_host_state.buttons |= GAMEPAD_MASK_L3;
            if (controller_report.buttonR3)       _controller_host_state.buttons |= GAMEPAD_MASK_R3;
            if (controller_report.buttonTouchpad) _controller_host_state.buttons |= GAMEPAD_MASK_A2;

            _controller_host_state.dpad = 0;
            if (controller_report.dpad == PS4_HAT_UP)    _controller_host_state.dpad |= GAMEPAD_MASK_UP;
            if (controller_report.dpad == PS4_HAT_RIGHT) _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;
            if (controller_report.dpad == PS4_HAT_DOWN)  _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;
            if (controller_report.dpad == PS4_HAT_LEFT)  _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;

            if (controller_report.buttonNorth) _controller_host_state.buttons |= GAMEPAD_MASK_B4;
            if (controller_report.buttonEast)  _controller_host_state.buttons |= GAMEPAD_MASK_B2;
            if (controller_report.buttonSouth) _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        }
    }

    prev_ds_report = controller_report;
}

void GamepadUSBHostListener::update_ctrlr() {
    if (controller_pid == DS4_ORG_PRODUCT_ID || controller_pid == DS4_PRODUCT_ID ||
        controller_pid == PS4_PRODUCT_ID     || controller_pid == PS4_WHEEL_PRODUCT_ID ||
        controller_pid == 0xB67B             || controller_pid == 0x00EE) {
        if (isDS4Identified) update_ds4();
    }
}

void GamepadUSBHostListener::update_ds4() {}

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
}

void GamepadUSBHostListener::get_report_complete(uint8_t, uint8_t, uint8_t report_id, uint8_t, uint16_t) {
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
    result = diff_than_2(rpt1->leftStickX,  rpt2->leftStickX)  ||
             diff_than_2(rpt1->leftStickY,  rpt2->leftStickY)  ||
             diff_than_2(rpt1->rightStickX, rpt2->rightStickX) ||
             diff_than_2(rpt1->rightStickY, rpt2->rightStickY);
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
