# Manual stroboscopic belt tuning - continuous, Prusa-style, self-contained.
#
# EXPERIMENTAL. This module owns BOTH halves of Prusa's manual_belt_tuning:
#   * the strobe LED (software or hardware PWM, run-time variable frequency),
#   * the continuous toolhead excitation + the Mainsail prompt "knob".
# So you configure the LED pin directly here - no separate [strobe] section.
#
# How it works (mirrors Prusa's resonate() loop):
#   * a reactor timer keeps the move queue topped up with short oscillation
#     segments, each generated at the *current* target frequency, so a
#     frequency change takes effect within ~one segment (no stutter);
#   * the SET/STEP commands only update an attribute + the strobe and return
#     immediately, so the UI stays responsive (no blocking g-code);
#   * the strobe is driven directly (register pokes / soft pwm), in sync.
#
# Config:
#   [belt_tune]
#   # --- strobe / LED ---
#   pin: PB11
#   hardware_pwm: True
#   #timer_clock_hz: 90000000   ; exact timer clock (F446 TIM2/3/4/5 = 90e6)
#   #strobe_duty: 0.137         ; LED duty (0..1); default ~13.7% like Prusa
#   #strobe_offset_hz: 2.0      ; strobe = excite freq + offset (slow drift)
#   # --- excitation ---
#   axis: a                     ; x / y / a / b
#   #accel_per_hz: 75.0
#   #start_frequency: 87.0
#   #freq_min: 40
#   #freq_max: 200
#   #z_height: 30
#   #point: 150,150            ; X,Y tuning position (default: bed center)
#   #lookahead_time: 0.5        ; seconds of motion kept queued (< 2.0!)
#   #tick_time: 0.15
#   #step_small: 0.5
#   #step_big: 2.0
#   #target_frequency: 110.0
#   #tolerance_hz: 1.0
#   #belt_mass_kg_m: 0.0        ; optional tension readout (string formula)
#   #belt_length_m: 0.0
#
# Commands:
#   BELT_TUNE_START [AXIS=] [FREQUENCY=] [TARGET=]
#   BELT_TUNE_STEP DELTA=0.5
#   BELT_TUNE_SET FREQUENCY=93.5
#   BELT_TUNE_STOP            (also: BELT_TUNE_ABORT)
#   STROBE_STATUS            (diagnostics: live PWM state)
#
# Copyright (C) 2026 - GNU GPLv3

import logging
import math

from . import pwm_cycle_time

######################################################################
# Strobe driver (embedded; soft + hardware PWM with run-time frequency)
######################################################################

# Hardware timer register offsets (STM32 generic/advanced timers)
TIMER_EGR = 0x14
TIMER_PSC = 0x28
TIMER_ARR = 0x2C
TIMER_CCR1 = 0x34
TIM_EGR_UG = 0x01
ORDER_16 = 1
ORDER_32 = 2

# Prusa's strobe duty (35/255 ~ 13.7%): more dark than light for clarity.
PRUSA_STROBE_DUTY = 35.0 / 255.0

# pin -> (timer_base, channel) for STM32F446 (from src/stm32/hard_pwm.c)
STM32F446_PWM_PINS = {
    'PA5': (0x40000000, 1), 'PA15': (0x40000000, 1),
    'PB3': (0x40000000, 2), 'PB10': (0x40000000, 3),
    'PB11': (0x40000000, 4), 'PB2': (0x40000000, 4),
    'PB4': (0x40000400, 1), 'PB5': (0x40000400, 2),
    'PB0': (0x40000400, 3), 'PB1': (0x40000400, 4),
    'PB6': (0x40000800, 1), 'PD12': (0x40000800, 1),
    'PB7': (0x40000800, 2), 'PD13': (0x40000800, 2),
    'PD14': (0x40000800, 3), 'PD15': (0x40000800, 4),
    'PA0': (0x40000C00, 1), 'PA1': (0x40000C00, 2),
    'PA2': (0x40000C00, 3), 'PA3': (0x40000C00, 4),
    'PA8': (0x40010000, 1), 'PA9': (0x40010000, 2),
    'PA10': (0x40010000, 3), 'PA11': (0x40010000, 4),
    'PC6': (0x40010400, 1), 'PC7': (0x40010400, 2),
    'PC8': (0x40010400, 3), 'PC9': (0x40010400, 4),
}


class StrobeDriver:
    # init_freq is used to program the hardware timer / soft cycle at startup;
    # the LED starts OFF (duty 0) until belt tuning enables it.
    def __init__(self, printer, config, init_freq):
        self.printer = printer
        self.gcode = printer.lookup_object('gcode')
        self.hardware_pwm = config.getboolean('hardware_pwm', False)
        self.init_freq = init_freq
        pin_desc = config.get('pin')
        if self.hardware_pwm:
            self._init_hardware(config, pin_desc)
        else:
            self._init_software(config, pin_desc)

    def apply(self, freq, duty):
        if self.hardware_pwm:
            self._apply_hardware(freq, duty)
        else:
            self._apply_software(freq, duty)

    def off(self):
        self.apply(0.0, 0.0)

    # ---- software backend ----
    def _init_software(self, config, pin_desc):
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(pin_desc, can_invert=True)
        self.mcu = pin_params['chip']
        self.soft_pin = pwm_cycle_time.MCU_pwm_cycle(
            pin_params, 1.0 / self.init_freq, 0.0, 0.0)
        self.last_duty = 0.0
        self.last_cycle = 1.0 / self.init_freq
        self.last_pt = 0.0

    def _apply_software(self, freq, duty):
        d = 0.0 if freq <= 0.0 else max(0.0, min(1.0, duty))
        cyc = 1.0 / freq if freq > 0.0 else self.last_cycle
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(
            lambda pt: self._set_software(pt, d, cyc))

    def _set_software(self, print_time, duty, cyc):
        if duty == self.last_duty and cyc == self.last_cycle:
            return
        get_mst = getattr(self.mcu, 'min_schedule_time', None)
        mst = get_mst() if get_mst is not None else 0.100
        print_time = max(print_time, self.last_pt + mst)
        self.soft_pin.set_pwm_cycle(print_time, duty, cyc)
        self.last_duty = duty
        self.last_cycle = cyc
        self.last_pt = print_time

    # ---- hardware backend (register pokes via stock debug_write) ----
    def _init_hardware(self, config, pin_desc):
        self.invert = pin_desc.startswith('!')
        pin_name = pin_desc.lstrip('!^').strip()
        if pin_name not in STM32F446_PWM_PINS:
            raise config.error(
                "belt_tune: pin '%s' is not a known STM32F446 hardware PWM"
                " pin (extend STM32F446_PWM_PINS)" % pin_name)
        self.timer_base, self.channel = STM32F446_PWM_PINS[pin_name]
        self.cfg_timer_clk = config.getfloat('timer_clock_hz', 0.0, minval=0.)
        self.timer_clk = None
        ppins = self.printer.lookup_object('pins')
        self.hw_pin = ppins.setup_pin('pwm', pin_desc)
        self.mcu = self.hw_pin.get_mcu()
        self.hw_pin.setup_max_duration(0.0)
        self.hw_pin.setup_cycle_time(1.0 / self.init_freq, True)
        self.hw_pin.setup_start_value(0.0, 0.0)  # start OFF
        self.debug_read = self.debug_write = None
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)

    def _handle_connect(self):
        get_constants = getattr(self.mcu, 'get_constants', None)
        mcu_type = get_constants().get('MCU', '') if get_constants else ''
        if mcu_type and not mcu_type.startswith('stm32f446'):
            logging.warning("belt_tune: register map assumes STM32F446,"
                            " MCU reports '%s'", mcu_type)
        self.debug_read = self.mcu.lookup_query_command(
            "debug_read order=%c addr=%u", "debug_result val=%u")
        self.debug_write = self.mcu.lookup_command(
            "debug_write order=%c addr=%u val=%u")
        psc = self._read_reg(TIMER_PSC, ORDER_16)
        arr = self._read_reg(TIMER_ARR, ORDER_32)
        if self.cfg_timer_clk:
            self.timer_clk = self.cfg_timer_clk
        else:
            self.timer_clk = float((psc + 1) * (arr + 1)) * self.init_freq
        logging.info("belt_tune strobe: timer_clk=%.0f Hz (PSC=%d ARR=%d)",
                     self.timer_clk, psc, arr)
        self._apply_hardware(self.init_freq, 0.0)  # ensure OFF, fine ARR

    def _read_reg(self, offset, order):
        return self.debug_read.send([order, self.timer_base + offset])['val']

    def _write_reg(self, offset, order, val):
        self.debug_write.send([order, self.timer_base + offset,
                               val & 0xffffffff])

    def _apply_hardware(self, freq, duty):
        ccr_off = TIMER_CCR1 + (self.channel - 1) * 4
        if freq <= 0.0:
            self._write_reg(ccr_off, ORDER_32, 0)
            return
        total = max(2, int(round(self.timer_clk / freq)))
        psc = max(1, (total + 0xFFFF) // 0x10000)
        arr = max(2, min(0x10000, int(round(total / float(psc)))))
        d = max(0.0, min(1.0, duty))
        if self.invert:
            d = 1.0 - d
        ccr = max(0, min(arr, int(round(d * arr))))
        self._write_reg(TIMER_ARR, ORDER_32, arr - 1)
        self._write_reg(ccr_off, ORDER_32, ccr)
        self._write_reg(TIMER_PSC, ORDER_16, psc - 1)
        self._write_reg(TIMER_EGR, ORDER_16, TIM_EGR_UG)

    def status_text(self):
        if self.hardware_pwm and self.debug_read is not None:
            psc = self._read_reg(TIMER_PSC, ORDER_16)
            arr = self._read_reg(TIMER_ARR, ORDER_32)
            ccr = self._read_reg(TIMER_CCR1 + (self.channel - 1) * 4, ORDER_32)
            period = (psc + 1) * (arr + 1)
            freq = self.timer_clk / period if period else 0.0
            duty = ccr / float(arr + 1) if arr else 0.0
            if self.invert:
                duty = 1.0 - duty
            return ("strobe (hardware): PSC=%d ARR=%d CCR=%d -> %.2f Hz,"
                    " duty %.1f%%" % (psc, arr, ccr, freq, duty * 100.))
        return ("strobe (software): %.2f Hz, duty %.3f"
                % (1.0 / self.last_cycle if self.last_cycle else 0.0,
                   self.last_duty))


######################################################################
# Belt tuning wizard (continuous excitation + prompt)
######################################################################

AXES = {
    'x': (1.0, 0.0, 0.0),
    'y': (0.0, 1.0, 0.0),
    'a': (1.0, -1.0, 0.0),
    'b': (1.0, 1.0, 0.0),
}

MAX_LOOKAHEAD = 1.0
MAX_SEGMENTS_PER_TICK = 200
MAX_OFFSET_MM = 5.0


class BeltTune:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        axis = config.get('axis', 'a').lower()
        if axis not in AXES:
            raise config.error("belt_tune: axis must be one of x/y/a/b")
        self.axis = axis

        self.accel_per_hz = config.getfloat('accel_per_hz', 75.0, above=0.)
        self.start_frequency = config.getfloat('start_frequency', 87.0, above=0.)
        self.strobe_offset = config.getfloat('strobe_offset_hz', 2.0)
        self.strobe_duty = config.getfloat('strobe_duty', PRUSA_STROBE_DUTY,
                                           minval=0., maxval=1.)
        self.freq_min = config.getfloat('freq_min', 40.0, above=0.)
        self.freq_max = config.getfloat('freq_max', 200.0, above=self.freq_min)
        self.z_height = config.getfloat('z_height', 30.0, above=0.)
        self.lookahead_time = config.getfloat('lookahead_time', 0.5,
                                              above=0., maxval=MAX_LOOKAHEAD)
        self.tick_time = config.getfloat('tick_time', 0.15, above=0.01)
        self.step_small = config.getfloat('step_small', 0.5, above=0.)
        self.step_big = config.getfloat('step_big', 2.0, above=0.)
        self.target_freq = config.getfloat('target_frequency', 110.0, above=0.)
        self.tolerance_hz = config.getfloat('tolerance_hz', 1.0, above=0.)
        self.belt_mass_kg_m = config.getfloat('belt_mass_kg_m', 0.0, minval=0.)
        self.belt_length_m = config.getfloat('belt_length_m', 0.0, minval=0.)
        # Peak-to-peak/2 displacement (mm) for the direct-stepper hybrid burst.
        # Keep small - the CoreXY motors are held, so this elastically loads
        # the belts; too large risks skipped steps on the Y motors.
        self.hybrid_amplitude_mm = config.getfloat('hybrid_amplitude_mm', 0.05,
                                                   above=0.)

        point = config.get('point', None)
        self.point_xy = None
        if point is not None:
            try:
                x, y = (float(v) for v in point.split(','))
                self.point_xy = (x, y)
            except Exception:
                raise config.error("belt_tune: 'point' must be 'X,Y'")

        # Embedded strobe driver (programs the timer at start_frequency)
        self.strobe = StrobeDriver(self.printer, config, self.start_frequency)

        # Runtime / generator state
        self.active = False
        self.freq = self.start_frequency
        self.direction = self._normalize(AXES[axis])
        self.base = None
        self.saved_accel = None
        self.saved_mcr = None
        self._shaper = None
        self._queued_pt = 0.0
        self._s = 0.0
        self._last_v = 0.0
        self._last_v2 = 0.0
        self._sign = 1.0
        self._sub = 0
        self.toolhead = None
        self.feed_timer = self.reactor.register_timer(self._feed_timer)

        self.gcode.register_command('BELT_TUNE_START', self.cmd_START,
                                    desc='Start continuous stroboscopic belt tuning')
        self.gcode.register_command('BELT_TUNE_STEP', self.cmd_STEP,
                                    desc='Nudge the tuning frequency by DELTA Hz')
        self.gcode.register_command('BELT_TUNE_SET', self.cmd_SET,
                                    desc='Set the tuning frequency to FREQUENCY Hz')
        self.gcode.register_command('BELT_TUNE_STOP', self.cmd_STOP,
                                    desc='Stop belt tuning')
        self.gcode.register_command('BELT_TUNE_ABORT', self.cmd_STOP,
                                    desc='Stop belt tuning')
        self.gcode.register_command('STROBE_STATUS', self.cmd_STROBE_STATUS,
                                    desc='Report the strobe live PWM state')
        self.gcode.register_command('BELT_TUNE_HYBRID_BURST',
                                    self.cmd_HYBRID_BURST,
                                    desc='EXPERIMENTAL: vibrate the hybrid Y '
                                         'motor(s) directly while CoreXY motors '
                                         'are held (single-frequency burst)')

    @staticmethod
    def _normalize(d):
        m = math.sqrt(sum(c * c for c in d))
        return tuple(c / m for c in d)

    # ------------------------------------------------------------------
    # Oscillation generator
    # ------------------------------------------------------------------
    def _pos(self, s):
        return [self.base[0] + s * self.direction[0],
                self.base[1] + s * self.direction[1],
                self.base[2] + s * self.direction[2],
                self.base[3]]

    def _queue_one_segment(self):
        f = min(self.freq_max, max(self.freq_min, self.freq))
        a_mag = self.accel_per_hz * f
        t_seg = 0.25 / f

        if self._sub == 0:
            accel = self._sign * a_mag
            self._sub = 1
        else:
            accel = -self._sign * a_mag
            self._sub = 0
            self._sign = -self._sign

        v = self._last_v + accel * t_seg
        if abs(v) < 1e-6:
            v = 0.0
        v2 = v * v
        half_inv = 0.5 / accel if accel else 0.0
        d = (v2 - self._last_v2) * half_inv
        new_s = self._s + d
        abs_v = abs(v)
        abs_last_v = abs(self._last_v)

        if abs(new_s) > MAX_OFFSET_MM:
            raise self.gcode.error('belt_tune: oscillation drifted too far '
                                   '(%.2f mm) - aborting' % (new_s,))

        if v * self._last_v < 0.0:
            s_zero = self._s + (-self._last_v2 * half_inv)
            self.toolhead.move(self._pos(s_zero), max(abs_last_v, 1e-3))
            self.toolhead.move(self._pos(new_s), max(abs_v, 1e-3))
        else:
            self.toolhead.move(self._pos(new_s), max(abs_v, abs_last_v, 1e-3))

        self._s = new_s
        self._last_v = v
        self._last_v2 = v2
        self._queued_pt += t_seg

    def _feed_timer(self, eventtime):
        if not self.active:
            return self.reactor.NEVER
        try:
            est = self.toolhead.mcu.estimated_print_time(eventtime)
            count = 0
            while (self._queued_pt - est < self.lookahead_time
                   and count < MAX_SEGMENTS_PER_TICK):
                self._queue_one_segment()
                count += 1
        except Exception as e:
            logging.exception('belt_tune: feed error')
            self._teardown()
            self.gcode.respond_info('belt_tune stopped on error: %s' % (e,))
            return self.reactor.NEVER
        return eventtime + self.tick_time

    # ------------------------------------------------------------------
    # Start / stop
    # ------------------------------------------------------------------
    def _strobe_to(self, freq):
        if freq > 0:
            self.strobe.apply(freq + self.strobe_offset, self.strobe_duty)
        else:
            self.strobe.off()

    def _teardown(self):
        if not self.active:
            return
        self.active = False
        self.reactor.update_timer(self.feed_timer, self.reactor.NEVER)
        try:
            self.toolhead.wait_moves()
        except Exception:
            logging.exception('belt_tune: wait_moves on teardown')
        # Re-enable input shaping if we disabled it
        if self._shaper is not None:
            self._shaper.enable_shaping()
            self._shaper = None
        if self.saved_accel is not None:
            if self.saved_mcr is not None:
                self.gcode.run_script_from_command(
                    'SET_VELOCITY_LIMIT ACCEL=%.0f MINIMUM_CRUISE_RATIO=%.3f'
                    % (self.saved_accel, self.saved_mcr))
            else:
                self.gcode.run_script_from_command(
                    'SET_VELOCITY_LIMIT ACCEL=%.0f' % (self.saved_accel,))
        self.saved_accel = self.saved_mcr = None

    def cmd_START(self, gcmd):
        if self.active:
            raise gcmd.error('belt_tune already running; STOP it first')
        self.toolhead = self.printer.lookup_object('toolhead')

        axis = gcmd.get('AXIS', self.axis).lower()
        if axis not in AXES:
            raise gcmd.error('AXIS must be one of x/y/a/b')
        self.axis = axis
        self.direction = self._normalize(AXES[axis])
        self.freq = gcmd.get_float('FREQUENCY', self.start_frequency, above=0.)
        self.target_freq = gcmd.get_float('TARGET', self.target_freq, above=0.)

        st = self.toolhead.get_status(self.reactor.monotonic())
        if 'xyz' not in st['homed_axes']:
            self.gcode.run_script_from_command('G28')
            st = self.toolhead.get_status(self.reactor.monotonic())

        if self.point_xy is not None:
            px, py = self.point_xy
        else:
            px = (st['axis_minimum'][0] + st['axis_maximum'][0]) / 2.0
            py = (st['axis_minimum'][1] + st['axis_maximum'][1]) / 2.0
        self.toolhead.manual_move([px, py, self.z_height], 120.0)
        self.toolhead.wait_moves()

        st = self.toolhead.get_status(self.reactor.monotonic())
        self.saved_accel = st['max_accel']
        self.saved_mcr = st.get('minimum_cruise_ratio')
        tuning_accel = self.accel_per_hz * self.freq_max
        if self.saved_mcr is not None:
            self.gcode.run_script_from_command(
                'SET_VELOCITY_LIMIT ACCEL=%.0f MINIMUM_CRUISE_RATIO=0'
                % (tuning_accel,))
        else:
            self.gcode.run_script_from_command(
                'SET_VELOCITY_LIMIT ACCEL=%.0f' % (tuning_accel,))

        # Disable input shaping so the excitation isn't filtered/distorted
        # (Shake&Tune does the same during resonance testing).
        self._shaper = self.printer.lookup_object('input_shaper', None)
        if self._shaper is not None:
            self._shaper.disable_shaping()

        self.base = self.toolhead.get_position()
        self._queued_pt = self.toolhead.get_last_move_time()
        self._s = 0.0
        self._last_v = 0.0
        self._last_v2 = 0.0
        self._sign = 1.0
        self._sub = 0
        self.active = True
        self._strobe_to(self.freq)
        self.reactor.update_timer(self.feed_timer, self.reactor.NOW)
        self._render()

    def cmd_STEP(self, gcmd):
        if not self.active:
            raise gcmd.error('belt_tune is not running')
        self._set_freq(self.freq + gcmd.get_float('DELTA'))
        self._render()

    def cmd_SET(self, gcmd):
        if not self.active:
            raise gcmd.error('belt_tune is not running')
        self._set_freq(gcmd.get_float('FREQUENCY', above=0.))
        self._render()

    def _set_freq(self, f):
        self.freq = min(self.freq_max, max(self.freq_min, f))
        self._strobe_to(self.freq)

    def cmd_STOP(self, gcmd):
        f = self.freq
        was_active = self.active
        self._teardown()
        self.strobe.off()
        if was_active:
            self.gcode.respond_info(self._result_text(f))
        self.gcode.respond_raw('// action:prompt_end')

    def cmd_STROBE_STATUS(self, gcmd):
        gcmd.respond_info(self.strobe.status_text())

    # ------------------------------------------------------------------
    # EXPERIMENTAL: direct-stepper hybrid-Y burst
    # ------------------------------------------------------------------
    # Oscillates ONE hybrid Y motor directly by looping force_move.manual_move
    # (the exact, proven primitive behind STEPPER_BUZZ), leaving the CoreXY
    # motors untouched (they hold position). One belt at a time -> this also
    # lets you test the left/right Y belts individually, which kinematic moves
    # can't. An even number of half-cycles returns the motor to its start, so
    # no re-home is needed (unless steps are skipped). Blocking burst:
    #   BELT_TUNE_HYBRID_BURST FREQUENCY=87 DURATION=2 STEPPER=stepper_y
    #       [AMPLITUDE=0.05]   (AMPLITUDE in mm)
    def cmd_HYBRID_BURST(self, gcmd):
        if self.active:
            raise gcmd.error('stop the continuous belt_tune (BELT_TUNE_STOP) '
                             'before a hybrid burst')
        freq = gcmd.get_float('FREQUENCY', self.start_frequency, above=1.)
        duration = gcmd.get_float('DURATION', 2.0, above=0.1, maxval=30.)
        # force_move / manual_move work in millimetres (BUZZ uses dist=1.0=1mm)
        amp = gcmd.get_float('AMPLITUDE', self.hybrid_amplitude_mm, above=0.)
        name = gcmd.get('STEPPER', 'stepper_y').lower()
        name = {'y': 'stepper_y', 'y1': 'stepper_y1'}.get(name, name)
        if name not in ('stepper_y', 'stepper_y1'):
            raise gcmd.error('STEPPER must be stepper_y or stepper_y1 '
                             '(one hybrid belt at a time)')
        fm = self.printer.lookup_object('force_move', None)
        if fm is None:
            raise gcmd.error('[force_move] is required for the hybrid burst')
        stepper = fm.lookup_stepper(name)

        dist = 2.0 * amp             # travel per half-cycle (peak-to-peak)
        half = 0.5 / freq            # half period -> manual_move time = dist/speed
        speed = dist / half          # = 4 * amp * freq
        n_half = int(round(duration / half))
        n_half += n_half % 2         # even -> net-zero -> no re-home

        was_enable = fm._force_enable(stepper)
        self.strobe.apply(freq + self.strobe_offset, self.strobe_duty)
        try:
            for i in range(n_half):
                fm.manual_move(stepper, dist if i % 2 == 0 else -dist, speed)
        finally:
            self.strobe.off()
            fm._restore_enable(stepper, was_enable)

        gcmd.respond_info(
            'hybrid burst: %.1f Hz, %d half-cycles, amp %.3f mm on %s'
            % (freq, n_half, amp, name))

    # ------------------------------------------------------------------
    # Tension + prompt
    # ------------------------------------------------------------------
    def _tension(self, f):
        if self.belt_mass_kg_m > 0 and self.belt_length_m > 0:
            return 4.0 * self.belt_mass_kg_m * self.belt_length_m ** 2 * f * f
        return None

    def _result_text(self, f):
        t = self._tension(f)
        if t is not None:
            return ('belt_tune done: %.1f Hz (~%.1f N), target %.1f Hz'
                    % (f, t, self.target_freq))
        return 'belt_tune done: %.1f Hz, target %.1f Hz' % (f, self.target_freq)

    def _p(self, msg):
        self.gcode.respond_raw('// action:%s' % (msg,))

    def _render(self):
        f = self.freq
        tgt = self.target_freq
        delta = tgt - f
        self._p('prompt_begin Belt tune: %.1f Hz' % (f,))
        t = self._tension(f)
        if t is not None:
            self._p('prompt_text Current: %.1f Hz  ~%.1f N' % (f, t))
            self._p('prompt_text Target:  %.1f Hz  ~%.1f N'
                    % (tgt, self._tension(tgt)))
        else:
            self._p('prompt_text Current: %.1f Hz' % (f,))
            self._p('prompt_text Target:  %.1f Hz' % (tgt,))
        if abs(delta) <= self.tolerance_hz:
            self._p('prompt_text -> within tolerance (%.1f Hz). Tension OK.'
                    % (delta,))
        elif delta > 0:
            self._p('prompt_text -> too loose: tighten, need +%.1f Hz' % (delta,))
        else:
            self._p('prompt_text -> too tight: loosen, need %.1f Hz' % (delta,))
        self._p('prompt_button_group_start')
        self._p('prompt_button -%g|BELT_TUNE_STEP DELTA=-%g|secondary'
                % (self.step_big, self.step_big))
        self._p('prompt_button -%g|BELT_TUNE_STEP DELTA=-%g|secondary'
                % (self.step_small, self.step_small))
        self._p('prompt_button +%g|BELT_TUNE_STEP DELTA=%g|primary'
                % (self.step_small, self.step_small))
        self._p('prompt_button +%g|BELT_TUNE_STEP DELTA=%g|primary'
                % (self.step_big, self.step_big))
        self._p('prompt_button_group_end')
        self._p('prompt_footer_button Done|BELT_TUNE_STOP|primary')
        self._p('prompt_footer_button Abort|BELT_TUNE_ABORT|error')
        self._p('prompt_show')

    def get_status(self, eventtime):
        return {'active': self.active, 'frequency': self.freq,
                'target_frequency': self.target_freq, 'axis': self.axis}


def load_config(config):
    return BeltTune(config)
