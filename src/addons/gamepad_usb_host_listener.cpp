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

void GamepadUSBHostListener::update_ds5() {

    if (last_led_profile != current_profile) {
        ds5_led_needs_update = true;
    }

    if (!ds5_led_needs_update) return;
    if (getMillis() < ds5_led_retry_timer) return;

    uint8_t buf[47];
    memset(buf, 0, sizeof(buf));

    buf[1] = 0x14;
    buf[38] = 0x02;
    buf[41] = 0x01;
    buf[42] = 0x02;
    buf[43] = 0x04;

    if (current_profile == PROFILE_WARZONE) {

        buf[44] = LED_WARZONE_R;
        buf[45] = LED_WARZONE_G;
        buf[46] = LED_WARZONE_B;

    } else {

        buf[44] = LED_EAFC_R;
        buf[45] = LED_EAFC_G;
        buf[46] = LED_EAFC_B;
    }

    if (tuh_hid_send_report(_controller_dev_addr, _controller_instance, 0x02, buf, 47)) {

        last_led_profile = current_profile;
        ds5_led_needs_update = false;

    } else {

        ds5_led_retry_timer = getMillis() + 100;
    }
}

void GamepadUSBHostListener::update_ctrlr() {

    update_ds5();

    if (controller_pid == DS4_ORG_PRODUCT_ID || controller_pid == DS4_PRODUCT_ID ||
        controller_pid == PS4_PRODUCT_ID || controller_pid == PS4_WHEEL_PRODUCT_ID ||
        controller_pid == 0xB67B || controller_pid == 0x00EE) {

        if (isDS4Identified)
            update_ds4();
    }
}

void GamepadUSBHostListener::update_ds4() {}

bool GamepadUSBHostListener::host_get_report(uint8_t report_id, void* report, uint16_t len) {

    awaiting_cb = true;

    return tuh_hid_get_report(
        _controller_dev_addr,
        _controller_instance,
        report_id,
        HID_REPORT_TYPE_FEATURE,
        report,
        len
    );
}

bool GamepadUSBHostListener::host_set_report(uint8_t report_id, void* report, uint16_t len) {

    awaiting_cb = true;

    return tuh_hid_set_report(
        _controller_dev_addr,
        _controller_instance,
        report_id,
        HID_REPORT_TYPE_FEATURE,
        report,
        len
    );
}

void GamepadUSBHostListener::set_report_complete(uint8_t, uint8_t, uint8_t, uint8_t, uint16_t) {

    awaiting_cb = false;
}

void GamepadUSBHostListener::get_report_complete(uint8_t, uint8_t, uint8_t report_id, uint8_t, uint16_t) {

    if (!isDS4Identified) {

        if (report_id == PS4AuthReport::PS4_DEFINITION)
            setup_ds4();
    }

    awaiting_cb = false;
}

uint32_t GamepadUSBHostListener::map(
    uint32_t x,
    uint32_t in_min,
    uint32_t in_max,
    uint32_t out_min,
    uint32_t out_max
) {

    return (x - in_min) * (out_max - out_min)
           / (in_max - in_min)
           + out_min;
}

bool GamepadUSBHostListener::diff_than_2(uint8_t x, uint8_t y) {

    return (x - y > 2) || (y - x > 2);
}

bool GamepadUSBHostListener::diff_report(PS4Report const* rpt1, PS4Report const* rpt2) {

    bool result;

    result = diff_than_2(rpt1->leftStickX, rpt2->leftStickX) ||
             diff_than_2(rpt1->leftStickY, rpt2->leftStickY) ||
             diff_than_2(rpt1->rightStickX, rpt2->rightStickX) ||
             diff_than_2(rpt1->rightStickY, rpt2->rightStickY);

    result |= memcmp(
        &rpt1->rightStickY + 1,
        &rpt2->rightStickY + 1,
        sizeof(PS4Report) - 6
    );

    return result;
}

void GamepadUSBHostListener::setup_ds4() {

    if (hasDS4DefReport)
        memcpy(&ds4Config, report_buffer + 1, sizeof(PS4ControllerConfig));

    if ((ds4Config.hidUsage == 0x2721) || (ds4Config.hidUsage == 0x2127))
        isDS4Identified = true;
}

void GamepadUSBHostListener::init_ds4(
    const uint8_t* descReport,
    uint16_t descLen
) {

    isDS4Identified = false;

    tuh_hid_report_info_t report_info[4];

    uint8_t report_count =
        tuh_hid_parse_report_descriptor(
            report_info,
            4,
            descReport,
            descLen
        );

    for(uint8_t i = 0; i < report_count; i++) {

        if (report_info[i].report_id ==
            PS4AuthReport::PS4_DEFINITION) {

            memset(report_buffer, 0, PS4_ENDPOINT_SIZE);

            report_buffer[0] =
                PS4AuthReport::PS4_DEFINITION;

            host_get_report(
                PS4AuthReport::PS4_DEFINITION,
                report_buffer,
                48
            );

            hasDS4DefReport = true;

            break;
        }
    }
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
        case PS4_PRODUCT_ID:
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

        if (diff_report(&prev_report, &controller_report) || macro_mute_active || turbo_state) {

            _controller_host_state.lx = map(controller_report.leftStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ly = map(controller_report.leftStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.rx = map(controller_report.rightStickX,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);
            _controller_host_state.ry = map(controller_report.rightStickY,0,255,GAMEPAD_JOYSTICK_MIN,GAMEPAD_JOYSTICK_MAX);

            _controller_host_state.lt = 0;
            _controller_host_state.rt = 0;
            _controller_host_state.buttons = 0;

            _controller_host_analog = true;

            if (controller_report.buttonSelect && controller_report.buttonStart) {

                if (!profile_switch_held) {
                    profile_switch_held = true;
                    profile_switch_timer = getMillis();
                }
                else if (getMillis() - profile_switch_timer > 2000) {

                    current_profile = (current_profile == PROFILE_EAFC) ? PROFILE_WARZONE : PROFILE_EAFC;

                    profile_switch_held = false;
                    profile_switch_timer = 0;

                    ds5_led_needs_update = true;
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

                        if (getMillis() - square_hold_start < 245) {

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

                if (controller_report.buttonR1) _controller_host_state.rt = 255;
                if (controller_report.buttonL1) _controller_host_state.buttons |= GAMEPAD_MASK_L1;

                _controller_host_state.lt = controller_report.rightTrigger;

                if (controller_report.leftTrigger > 160)
                    _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                if (controller_report.buttonSelect)
                    _controller_host_state.buttons |= GAMEPAD_MASK_S1;

                if (controller_report.buttonStart)
                    _controller_host_state.buttons |= GAMEPAD_MASK_S2;
            }

            else {

                if (controller_report.rightTrigger > 200 && controller_report.leftTrigger > 200) {

                    uint32_t recoil_val = _controller_host_state.ry + ANTI_RECOIL_STRENGTH;

                    if (recoil_val > GAMEPAD_JOYSTICK_MAX)
                        recoil_val = GAMEPAD_JOYSTICK_MAX;

                    _controller_host_state.ry = recoil_val;
                }

                if (controller_report.buttonL1) {

                    if (getMillis() - turbo_timer > 40) {

                        turbo_state = !turbo_state;
                        turbo_timer = getMillis();
                    }

                    if (turbo_state)
                        _controller_host_state.buttons |= GAMEPAD_MASK_B1;

                } else {
                    turbo_state = false;
                }

                if (controller_report.buttonWest)
                    _controller_host_state.buttons |= GAMEPAD_MASK_B3;

                if (controller_report.buttonSelect && !controller_report.buttonStart)
                    _controller_host_state.buttons |= GAMEPAD_MASK_L1;

                if (controller_report.buttonR1)
                    _controller_host_state.buttons |= GAMEPAD_MASK_R1;

                if (controller_report.buttonStart)
                    _controller_host_state.buttons |= GAMEPAD_MASK_S2;

                if (controller_report.buttonHome)
                    _controller_host_state.buttons |= GAMEPAD_MASK_A1;

                _controller_host_state.lt = controller_report.leftTrigger;
                _controller_host_state.rt = controller_report.rightTrigger;
            }

            if (controller_report.buttonL3)
                _controller_host_state.buttons |= GAMEPAD_MASK_L3;

            if (controller_report.buttonR3)
                _controller_host_state.buttons |= GAMEPAD_MASK_R3;

            if (controller_report.buttonTouchpad)
                _controller_host_state.buttons |= GAMEPAD_MASK_A2;

            _controller_host_state.dpad = 0;

            if (controller_report.dpad == PS4_HAT_UP)
                _controller_host_state.dpad |= GAMEPAD_MASK_UP;

            if (controller_report.dpad == PS4_HAT_RIGHT)
                _controller_host_state.dpad |= GAMEPAD_MASK_RIGHT;

            if (controller_report.dpad == PS4_HAT_DOWN)
                _controller_host_state.dpad |= GAMEPAD_MASK_DOWN;

            if (controller_report.dpad == PS4_HAT_LEFT)
                _controller_host_state.dpad |= GAMEPAD_MASK_LEFT;

            if (controller_report.buttonNorth)
                _controller_host_state.buttons |= GAMEPAD_MASK_B4;

            if (controller_report.buttonEast)
                _controller_host_state.buttons |= GAMEPAD_MASK_B2;

            if (controller_report.buttonSouth)
                _controller_host_state.buttons |= GAMEPAD_MASK_B1;
        }
    }

    prev_report = controller_report;
}
