#include <Arduino.h>
#include <avr/io.h>

#include <Adafruit_NeoPixel.h>

#define RGB_PIN PB2
#define NUM_PIXELS 1

Adafruit_NeoPixel strip(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);


// ----------- CONFIG -----------

#define CODE_TURN_ON   0xA5
#define CODE_TURN_OFF  0x5A

#define ON_PERIOD_MIN     180    // after this, switch to OFF phase until power cycle

// ON phase behavior
#define ON_FLOOD_DURATION      60000UL        // 1 minute
#define ON_FLOOD_INTERVAL      (10UL * 60000UL)  // every 10 minutes

// OFF phase behavior
#define OFF_FLOOD_DURATION     60000UL        // 1 minute
#define OFF_FLOOD_INTERVAL     (5UL * 60000UL)   // every 5 minutes

#define PROPAGATION_INTERVAL_MS   800UL
#define PROPAGATION_BURSTS          1

#define IR_TX PB1
#define IR_RX PB0
#define BUZZER PB3

#define EPSILON                 64
#define PHASE_STEP              1
#define JUMP_TO_FLASH_MARGIN    16

#define BUZZER_ENABLED          1
#define CHIRP_BASE_DELAY        175
#define CHIRP_STEPS             8
#define CHIRP_CYCLES_PER_STEP   20
#define CHIRP_DELAY_DECREMENT   5
#define CHIRP_MIN_DELAY         20
#define CHIRP_PAUSE_MS          10

#define OSC_TICK_US             2000UL
#define REFRACTORY_FLASH        20
#define REFRACTORY_TRIGGER      10

const uint8_t GRADIENT_START_R[4] = {255, 255,   0, 200};
const uint8_t GRADIENT_START_G[4] = {135, 255,   0, 210};
const uint8_t GRADIENT_START_B[4] = {  0,   0, 255,   0};

const uint8_t GRADIENT_END_R[4]   = {  0, 255,   0, 255};
const uint8_t GRADIENT_END_G[4]   = {  0,   0, 255, 110};
const uint8_t GRADIENT_END_B[4]   = {  0, 255,   0,   0};

uint8_t phase_index = 0;
uint16_t PHASE_MAX = 255;
uint32_t timer_us = 0;
uint32_t red_timer = 0;


// ----------- IR LOW-LEVEL -----------

void emit_pulse(uint16_t cycles) {
  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << IR_TX);
    delayMicroseconds(15);
    PORTB &= ~(1 << IR_TX);
    delayMicroseconds(15);
  }
}

static inline void mark_us(uint16_t us) {
  uint16_t cycles = us / 30;
  if (cycles < 1) cycles = 1;
  emit_pulse(cycles);
}

static inline void space_us(uint16_t us) {
  PORTB &= ~(1 << IR_TX);
  delayMicroseconds(us);
}


// ----------- FRAME FORMAT -----------

void send_code(uint8_t v) {

  uint8_t inv = ~v;

  // START
  mark_us(6000);
  space_us(3000);

  // First byte
  for (int8_t i = 7; i >= 0; i--) {
    if (v & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  // Second byte (inverted)
  for (int8_t i = 7; i >= 0; i--) {
    if (inv & (1 << i)) {
      mark_us(2000);
      space_us(2000);
    } else {
      mark_us(1000);
      space_us(3000);
    }
  }

  space_us(6000);
}


// ----------- FIREFLY BEHAVIOR -----------

void delay_us_custom(uint16_t us) {
  while (us--) {
    for (uint8_t i = 0; i < 3; i++) asm volatile("nop");
  }
}

void chirp(void) {
#if BUZZER_ENABLED
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
#else
  PORTB &= ~(1 << BUZZER);
#endif
}

void half_chirp() {
  strip.setPixelColor(0, strip.Color(0, 123, 0));
  strip.show();
  delay_us_custom(50);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

#if BUZZER_ENABLED
  for (uint8_t i = 0; i < 5; i++) {
    PORTB |= (1 << BUZZER);
    delay_us_custom(100);
    PORTB &= ~(1 << BUZZER);
    delay_us_custom(100);
  }
#else
  PORTB &= ~(1 << BUZZER);
#endif
}

void set_fade_color(uint16_t phase, uint32_t /*diff*/) {
  uint8_t i = phase_index;

  uint8_t r = GRADIENT_START_R[i] + ((uint16_t)(GRADIENT_END_R[i] - GRADIENT_START_R[i]) * phase) / PHASE_MAX;
  uint8_t g = GRADIENT_START_G[i] + ((uint16_t)(GRADIENT_END_G[i] - GRADIENT_START_G[i]) * phase) / PHASE_MAX;
  uint8_t b = GRADIENT_START_B[i] + ((uint16_t)(GRADIENT_END_B[i] - GRADIENT_START_B[i]) * phase) / PHASE_MAX;

  strip.setPixelColor(0, strip.Color(g, r, b));  // GRB
  strip.show();
}



// ----------- MAIN -----------

int main(void) {

  init();

  strip.begin();
  strip.clear();
  strip.show();


  DDRB |= (1 << IR_TX);
  DDRB &= ~(1 << IR_RX);
  DDRB |= (1 << BUZZER);
  PORTB |= (1 << IR_RX);
  PORTB &= ~(1 << IR_TX);
  PORTB &= ~(1 << BUZZER);

  bool in_on_phase = true;

  uint32_t phase_start = millis();
  uint32_t last_flood_start = 0;
  uint32_t last_command_send = millis() - PROPAGATION_INTERVAL_MS;
  uint16_t phase = 0;
  uint8_t last_rx_state = (PINB & (1 << IR_RX));
  uint8_t refractory = 0;
  bool half_chirped = false;

  timer_us = micros();

  while (1) {

    uint32_t now = millis();

    // ---------- Determine current phase ----------
    if (in_on_phase && now - phase_start >= ON_PERIOD_MIN * 60000UL) {
      in_on_phase = false;
      phase_start = now;
      last_flood_start = 0;   // reset scheduling
    }

    // ---------- Select flood parameters ----------
    uint32_t flood_interval =
      in_on_phase ? ON_FLOOD_INTERVAL : OFF_FLOOD_INTERVAL;

    uint32_t flood_duration =
      in_on_phase ? ON_FLOOD_DURATION : OFF_FLOOD_DURATION;

    uint8_t current_code =
      in_on_phase ? CODE_TURN_ON : CODE_TURN_OFF;

    // ---------- Should we start a new flood? ----------
    if (last_flood_start == 0 ||
        now - last_flood_start >= flood_interval) {

        last_flood_start = now;
    }

    bool in_flood_window = (now - last_flood_start < flood_duration);

    // ---------- Are we inside flood window? ----------
    if (in_flood_window &&
        (uint32_t)(now - last_command_send) >= PROPAGATION_INTERVAL_MS) {

      uint32_t t0 = micros();

      for (uint8_t i = 0; i < PROPAGATION_BURSTS; i++) {
        send_code(current_code);
      }

      if (in_on_phase) {
        timer_us += micros() - t0;
      }

      last_command_send = now;
    }

    if (in_on_phase) {
      uint32_t now_us = micros();
      if ((uint32_t)(now_us - timer_us) >= OSC_TICK_US) {
        timer_us = now_us;

        set_fade_color(phase, (now_us - red_timer));

        phase += PHASE_STEP;
        if (phase > PHASE_MAX) phase = PHASE_MAX;

        if ((phase >= (PHASE_MAX / 2)) && !half_chirped) {
          half_chirp();
          half_chirped = true;
        }

        if (phase >= PHASE_MAX) {
          if (!in_flood_window) {
            emit_pulse(200);
          }
          chirp();
          phase = 0;
          half_chirped = false;
          refractory = REFRACTORY_FLASH;
        }

        uint8_t current_rx = (PINB & (1 << IR_RX));

        if (!in_flood_window && last_rx_state && !current_rx && refractory == 0) {
          if (phase > (PHASE_MAX / 4)) {
            uint16_t delta = ((uint32_t)EPSILON * (PHASE_MAX - phase)) / PHASE_MAX;
            phase += delta;
            if (phase > PHASE_MAX) phase = PHASE_MAX;

            if ((PHASE_MAX - phase) < JUMP_TO_FLASH_MARGIN) {
              emit_pulse(200);
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
    } else {
      strip.clear();
      strip.show();
      PORTB &= ~(1 << BUZZER);
    }
  }

  return 0;
}
