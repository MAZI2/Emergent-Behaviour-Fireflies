/*
 * Firefly Synchronization Device
 *
 * States:
 *  - SLEEP: minimal activity, wakes on IR edges (PCINT0)
 *  - PROPAGATE_ON: keeps blinking/chirping (NO phase-shift listen, NO phase-shift TX),
 *                  and relays TURN_ON frames for PROPAGATE_ON_TIME
 *  - ACTIVE: full oscillator + phase-shift listening + flash TX
 *  - PROPAGATE_OFF: everything OFF (no LED/buzzer), only relays TURN_OFF frames for PROPAGATE_OFF_TIME
 *
 * Key fixes for “erratic” behavior during PROPAGATE_ON:
 *  1) Propagation TX (send_code) is blocking ~40-50ms per frame, which “steals time” from the 2ms tick loop.
 *     We compensate by advancing `timer` by exactly the time spent transmitting.
 *  2) While we transmit, we temporarily DISABLE the IR receiver pin-change interrupt to avoid self-reception
 *     (your own IR LED coupling into the TSOP and filling the pulse queue).
 */

#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

// ---- Propagation timing (DEFINE as requested) ----
#define PROPAGATE_ON_TIME       60000UL   // 60 seconds
#define PROPAGATE_OFF_TIME      20000UL   // 20 seconds

#define PROPAGATION_INTERVAL_MS   800UL   // send every X ms
#define PROPAGATION_BURSTS          1     // frames per interval

#define TURN_ON_COOLDOWN_MS 300000UL

// ---- Oscillator tick ----
#define OSC_TICK_US             2000UL    // 2ms update cadence

enum SystemState {
  STATE_SLEEP,
  STATE_PROPAGATE_ON,
  STATE_ACTIVE,
  STATE_PROPAGATE_OFF
};

SystemState system_state = STATE_SLEEP;

uint32_t state_timer = 0;
uint32_t last_propagation_send = 0;
uint32_t last_propagate_on_end = 0;


// ===== HARDWARE CONFIGURATION =====
#define NEOPIXEL_PIN    PB2
#define NUM_PIXELS      1
#define IR_TX           PB1
#define IR_RX           PB0
#define LED             PB2
#define BUZZER          PB3

// ===== BEHAVIOR PARAMETERS =====
#define EPSILON                 64
#define PHASE_STEP              1
#define JUMP_TO_FLASH_MARGIN    16

// ===== AUDIO PARAMETERS =====
#define CHIRP_BASE_DELAY        175
#define CHIRP_STEPS             8
#define CHIRP_CYCLES_PER_STEP   20
#define CHIRP_DELAY_DECREMENT   5
#define CHIRP_MIN_DELAY         20
#define CHIRP_PAUSE_MS          10

// ===== TIMING PARAMETERS =====
#define IR_PULSE_HALF_PERIOD    15
#define REFRACTORY_FLASH        20
#define REFRACTORY_TRIGGER      10

// --- pulse queue ---
#define PULSE_Q_SIZE 16
volatile uint16_t pulse_q[PULSE_Q_SIZE];
volatile uint8_t q_head = 0;
volatile uint8_t q_tail = 0;

volatile uint32_t last_edge_us = 0;
volatile uint8_t last_level = 1;   // TSOP idle HIGH

ISR(PCINT0_vect) {
  uint8_t level = (PINB & (1 << IR_RX)) ? 1 : 0;
  uint32_t now = micros();

  if (level != last_level) {
    // rising edge = LOW pulse ended
    if (last_level == 0 && level == 1) {
      uint16_t w = (uint16_t)(now - last_edge_us);

      uint8_t next = (q_head + 1) & (PULSE_Q_SIZE - 1);
      if (next != q_tail) {         // drop if full
        pulse_q[q_head] = w;
        q_head = next;
      }
    }

    last_edge_us = now;
    last_level = level;
  }
}

// ---- color gradients ----
const uint8_t GRADIENT_START_R[4] = {255, 255,   0, 200};
const uint8_t GRADIENT_START_G[4] = {135, 255,   0, 210};
const uint8_t GRADIENT_START_B[4] = {  0,   0, 255,   0};

const uint8_t GRADIENT_END_R[4]   = {  0, 255,   0, 255};
const uint8_t GRADIENT_END_G[4]   = {  0,   0, 255, 110};
const uint8_t GRADIENT_END_B[4]   = {  0, 255,   0,   0};

// ===== GLOBAL VARIABLES =====
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== Dynamic Phase Configuration =====
const uint16_t PHASE_SEQUENCE[] = {450, 450, 210, 350};
#define PHASE_SEQUENCE_LEN (sizeof(PHASE_SEQUENCE) / sizeof(PHASE_SEQUENCE[0]))
uint8_t phase_index = 0;
uint16_t PHASE_MAX = 255;

// ===== TIMING/STATE VARS =====
uint32_t timer_us = 0;     // oscillator tick reference (micros)
uint32_t red_timer = 0;    // kept (used in ACTIVE shading)
bool red = false;

// ===== FUNCTION PROTOTYPES =====
void setup_hardware(void);
void emit_ir_pulse(uint16_t cycles);
void delay_us_custom(uint16_t us);
void chirp(void);
void half_chirp(void);
void set_fade_color(uint16_t phase, uint32_t diff);
void send_code(uint8_t v);
void send_code_freeze(uint8_t v, bool compensate_timer);

// ======================= HARDWARE SETUP =======================
void setup_hardware(void) {
  // IR pins
  DDRB |= (1 << IR_TX);      // IR emitter output
  DDRB &= ~(1 << IR_RX);     // IR receiver input
  PORTB |= (1 << IR_RX);     // pull-up on receiver pin (TSOP is active low output, but pull-up ok)

  // LED/buzzer pins
  DDRB |= (1 << LED);
  DDRB |= (1 << BUZZER);
  PORTB &= ~(1 << LED);
  PORTB &= ~(1 << BUZZER);

  // Enable pin-change interrupt on PB0 (PCINT0)
  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PCINT0);

  sei();

  // NeoPixel
  strip.begin();
  strip.clear();
  strip.show();
}

// ======================= IR TX LOW LEVEL =======================
void emit_ir_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    delayMicroseconds(IR_PULSE_HALF_PERIOD);
    PORTB &= ~(1 << IR_TX);
    delayMicroseconds(IR_PULSE_HALF_PERIOD);
  }
}

static inline void mark_us(uint16_t us) {
  uint16_t cycles = us / 30;
  if (cycles < 1) cycles = 1;
  emit_ir_pulse(cycles);
}

static inline void space_us(uint16_t us) {
  PORTB &= ~(1 << IR_TX);
  delayMicroseconds(us);
}

// Frame format:
// Start: 6000 mark + 3000 space
// 0-bit: 1000 mark + 3000 space
// 1-bit: 2000 mark + 2000 space
// End:   6000 space
void send_code(uint8_t v) {
  mark_us(6000);
  space_us(3000);

  for (int8_t i = 7; i >= 0; i--) {
    if (v & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  space_us(6000);
}

/*
 * IMPORTANT FIX:
 * - disable PCINT during transmit to avoid self-reception filling pulse queue
 * - optionally compensate timer_us so oscillator time does not “jump” after long TX block
 */
void send_code_freeze(uint8_t v, bool compensate_timer) {
  // Disable pin-change interrupt on PB0 while we TX
  PCMSK &= ~(1 << PCINT0);

  uint32_t t0 = micros();
  send_code(v);
  uint32_t dt = micros() - t0;

  // Re-enable receiver PCINT
  PCMSK |= (1 << PCINT0);

  // Optional: compensate oscillator timer so it doesn't "catch up" violently
  if (compensate_timer) {
    timer_us += dt;
  }
}

// ======================= IR RX DECODE =======================
static inline bool pulse_pop(uint16_t &w) {
  if (q_tail == q_head) return false;
  w = pulse_q[q_tail];
  q_tail = (q_tail + 1) & (PULSE_Q_SIZE - 1);
  return true;
}

bool decode_frame(uint8_t &out) {
  static bool in_frame = false;
  static uint8_t bit_count = 0;
  static uint8_t value = 0;
  static uint32_t last_activity = 0;

  uint16_t w;
  while (pulse_pop(w)) {
    last_activity = micros();

    // START DETECT
    if (!in_frame) {
      if (w > 4500 && w < 9000) {   // ~6000us mark
        in_frame = true;
        bit_count = 0;
        value = 0;
      }
      continue;
    }

    // BIT DECODE (width of LOW mark)
    if (w < 300 || w > 3500) {
      in_frame = false;
      continue;
    }

    value <<= 1;
    if (w > 1500) value |= 1;  // 1-bit ~2000us, 0-bit ~1000us
    bit_count++;

    if (bit_count >= 8) {
      out = value;
      in_frame = false;
      return true;
    }
  }

  // TIMEOUT
  if (in_frame && (micros() - last_activity > 60000UL)) {
    in_frame = false;
  }
  return false;
}

// ======================= AUDIO / LED =======================
void delay_us_custom(uint16_t us) {
  while (us--) {
    for (uint8_t i = 0; i < 3; i++) asm volatile("nop");
  }
}

void chirp(void) {
  uint16_t delay_val = CHIRP_BASE_DELAY;
  for (uint8_t c = 0; c < CHIRP_STEPS; c++) {
    for (uint8_t i = 0; i < CHIRP_CYCLES_PER_STEP; i++) {
      PORTB |= (1 << BUZZER);
      delay_us_custom(delay_val);
      PORTB &= ~(1 << BUZZER);
      delay_us_custom(delay_val);
    }
    delay_val -= CHIRP_DELAY_DECREMENT;
    if (delay_val < CHIRP_MIN_DELAY) delay_val = CHIRP_MIN_DELAY;
    delay(CHIRP_PAUSE_MS);
  }
}

void half_chirp() {
  strip.setPixelColor(0, strip.Color(0, 123, 0));
  strip.show();
  delay_us_custom(50);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  for (uint8_t i = 0; i < 5; i++) {
    PORTB |= (1 << BUZZER);
    delay_us_custom(100);
    PORTB &= ~(1 << BUZZER);
    delay_us_custom(100);
  }
}

void set_fade_color(uint16_t phase, uint32_t /*diff*/) {
  uint8_t i = phase_index;

  uint8_t r = GRADIENT_START_R[i] + ((uint16_t)(GRADIENT_END_R[i] - GRADIENT_START_R[i]) * phase) / PHASE_MAX;
  uint8_t g = GRADIENT_START_G[i] + ((uint16_t)(GRADIENT_END_G[i] - GRADIENT_START_G[i]) * phase) / PHASE_MAX;
  uint8_t b = GRADIENT_START_B[i] + ((uint16_t)(GRADIENT_END_B[i] - GRADIENT_START_B[i]) * phase) / PHASE_MAX;

  strip.setPixelColor(0, strip.Color(g, r, b));  // GRB
  strip.show();
}

// ======================= MAIN =======================
int main(void) {
  init();   // Arduino core init (Timer0, micros/millis)
  sei();
  setup_hardware();

  uint16_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;
  bool half_chirped = false;

  timer_us = micros();

  while (1) {
    // ---------- STATE MACHINE ----------
    switch (system_state) {

      case STATE_SLEEP: {
        // Optional: keep LED/buzzer off explicitly
        strip.clear();
        strip.show();
        PORTB &= ~(1 << BUZZER);

        // IDLE sleep keeps timers running (micros/millis), wakes on PCINT0
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sleep_cpu();
        sleep_disable();
        break;
      }

      case STATE_PROPAGATE_ON: {

        // ---- Propagation sending (TURN_ON) ----
        if ((uint32_t)(millis() - last_propagation_send) >= PROPAGATION_INTERVAL_MS) {

          uint32_t t0 = micros();

          for (uint8_t i = 0; i < PROPAGATION_BURSTS; i++) {
              send_code_freeze(CODE_TURN_ON, false);
              delay(5);
          }

          uint32_t dt = micros() - t0;
          timer_us += dt; 

          last_propagation_send = millis();
        }



        // ---- IDENTICAL oscillator engine as ACTIVE ----
        uint32_t now = micros();
        uint32_t elapsed = now - timer_us;

        if ((uint32_t)(now - timer_us) >= OSC_TICK_US) {
            timer_us = now;

            phase += PHASE_STEP;


            if (phase > PHASE_MAX)
                phase = PHASE_MAX;

            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = (uint16_t)phase * 255 / PHASE_MAX;

            strip.setPixelColor(0, strip.Color(g, r, b));  // GRB order
            strip.show();
            
            //set_fade_color(phase, 0);

            if ((phase >= (PHASE_MAX / 2)) && !half_chirped) {
                half_chirp();
                half_chirped = true;
            }

            if (phase >= PHASE_MAX) {
                // NOTE: no emit_ir_pulse() here
                chirp();
                phase = 0;
                half_chirped = false;
            }
        }

        // ---- Transition to ACTIVE ----
        if ((uint32_t)(millis() - state_timer) >= PROPAGATE_ON_TIME) {
          system_state = STATE_ACTIVE;

          last_propagate_on_end = millis();   // ← remember when it ended

          timer_us = micros();
          half_chirped = false;
          refractory = 0;
        }


        break;
      }


      case STATE_ACTIVE:
        // handled in the “ACTIVE loop” below
        break;

      case STATE_PROPAGATE_OFF: {
        // Everything off visually & audibly
        strip.clear();
        strip.show();
        PORTB &= ~(1 << BUZZER);

        if ((uint32_t)(millis() - last_propagation_send) >= PROPAGATION_INTERVAL_MS) {
          for (uint8_t i = 0; i < PROPAGATION_BURSTS; i++) {
            // no need to compensate timer_us here, but we DO disable PCINT during TX to avoid self RX
            send_code_freeze(CODE_TURN_OFF, false);
            delay(5);
          }
          last_propagation_send = millis();
        }

        if ((uint32_t)(millis() - state_timer) >= PROPAGATE_OFF_TIME) {
          system_state = STATE_SLEEP;
        }
        break;
      }
    }

    // ---------- ALWAYS: decode TURN_ON / TURN_OFF frames ----------
    uint8_t rx;
    if (decode_frame(rx)) {
      if (rx == CODE_TURN_ON) {

        uint32_t now_ms = millis();

        bool recently_propagated =
          (last_propagate_on_end != 0) &&
          ((uint32_t)(now_ms - last_propagate_on_end) < TURN_ON_COOLDOWN_MS);

        if (!recently_propagated &&
            (system_state == STATE_SLEEP ||
            system_state == STATE_ACTIVE)) {

            system_state = STATE_PROPAGATE_ON;

            state_timer = now_ms;
            last_propagation_send = now_ms;

            timer_us = micros();
            phase = 0;
            half_chirped = false;
            refractory = 0;
        }
      }


      // -------- TURN OFF --------
      if (rx == CODE_TURN_OFF &&
          system_state == STATE_ACTIVE) {

          system_state = STATE_PROPAGATE_OFF;
          state_timer = millis();
          last_propagation_send = millis();
      }
    }

    // ---------- ACTIVE oscillator + phase shift listening ----------
    if (system_state == STATE_ACTIVE) {
      uint32_t now = micros();
      if ((uint32_t)(now - timer_us) >= OSC_TICK_US) {
        timer_us = now;

        set_fade_color(phase, (now - red_timer));

        // Advance phase
        phase += PHASE_STEP;
        if (phase > PHASE_MAX) phase = PHASE_MAX;

        if ((phase >= (PHASE_MAX / 2)) && !half_chirped) {
          half_chirp();
          half_chirped = true;
        }

        if (phase >= PHASE_MAX) {
          emit_ir_pulse(200);
          chirp();
          phase = 0;
          half_chirped = false;
          refractory = REFRACTORY_FLASH;
        }

        // Phase-shift listening (only in ACTIVE)
        uint8_t current_rx = (PINB & (1 << IR_RX));

        if (last_rx_state && !current_rx && refractory == 0) {
          if (phase > (PHASE_MAX / 4)) {
            uint16_t delta = ((uint32_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
            phase += delta;
            if (phase > PHASE_MAX) phase = PHASE_MAX;

            if ((PHASE_MAX - phase) < JUMP_TO_FLASH_MARGIN) {
              emit_ir_pulse(200);
              chirp();
              phase = 0;
              half_chirped = false;
              refractory = REFRACTORY_FLASH;
            } else {
              refractory = REFRACTORY_TRIGGER;
            }
          }
        }

        if (refractory > 0) refractory--;
        last_rx_state = current_rx;
      }
    }
  }

  return 0;
}
