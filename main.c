#include "midi_usb.h" //see the lufa_midi implementation
#include "spi.h"
#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>

//SYSEX_BEGIN, SYSEX_EDUMANUFID, xnormidibootloader, SYSEX_END
const static uint8_t reboot_sequence[] = {
  SYSEX_BEGIN, SYSEX_EDUMANUFID,
  'x', 'n', 'o', 'r', 'm', 'i', 'd', 'i', 'b', 'o', 'o', 't', 'l', 'o', 'a', 'd', 'e', 'r',
  SYSEX_END
};

//forward declarations of callbacks
void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val);
void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);
void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);
void sysex_callback(MidiDevice * device, uint16_t count, uint8_t byte0, uint8_t byte1, uint8_t byte2);

void update_dac(uint8_t addr, uint16_t value);
void dac_sel(bool sel, uint8_t dac);
void jump_to_bootloader(void);

#define setp(port, pin, on) \
  if (on) { port |= _BV(pin); } \
  else { port &= ~_BV(pin); }

#define DAC0_DDR DDRB
#define DAC0_ENABLE_PORT PORTB
#define DAC0_ENABLE_PIN PINB0

#define DAC1_DDR DDRD
#define DAC1_ENABLE_PORT PORTD
#define DAC1_ENABLE_PIN PORTD0

int main(void) {
  clock_prescale_set(clock_div_4);
  MidiDevice midi_device;

  //spi enables as outputs
  DAC0_DDR |= _BV(DAC0_ENABLE_PIN);
  DAC1_DDR |= _BV(DAC1_ENABLE_PIN);

  //digital out, d3 [oops], c6
  DDRC |= _BV(PINC6);
  DDRD |= _BV(PIND3);

  dac_sel(false, 0);
  dac_sel(false, 1);

  //setup the device
  midi_usb_init(&midi_device);
  setup_spi(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK2);

  //register callbacks
  midi_register_cc_callback(&midi_device, cc_callback);
  midi_register_noteon_callback(&midi_device, note_on_callback);
  midi_register_noteoff_callback(&midi_device, note_off_callback);
  midi_register_sysex_callback(&midi_device, sysex_callback);

  while(1){
    //processes input from the midi device
    //and calls the appropriate callbacks
    midi_device_process(&midi_device);
  }

  return 0; //never happens
}

void jump_to_bootloader(void) {
  asm volatile("jmp 0x7E00");  // Teensy 2.0
}

void update_dac(uint8_t addr, uint16_t value) {
  uint8_t out[2];
  out[0] = value >> 6;
  out[1] = value << 2;

  out[0] |= addr << 6; //address
  out[0] |= 0x3 << 4; //control [0b11]

  uint8_t dac = addr >> 2;
  dac_sel(true, dac);
  send_spi(out[0]);
  send_spi(out[1]);
  _delay_us(10);
  dac_sel(false, dac);
}

void update_digital(uint8_t addr, bool on) {
  switch (addr) {
    case 0:
      setp(PORTD, PD3, on);
      break;
    case 1:
      setp(PORTC, PC6, on);
      break;
    default:
      break;
  }
}

void dac_sel(bool sel, uint8_t dac) {
  if (dac == 0) {
    setp(DAC0_ENABLE_PORT, DAC0_ENABLE_PIN, !sel);
  } else {
    setp(DAC1_ENABLE_PORT, DAC1_ENABLE_PIN, !sel);
  }
}

void note_callback(bool on, uint8_t chan, uint8_t note, uint8_t vel) {
  if (chan < 8) {
    if (note > 120)
      return; //cannot get there

    uint16_t v = note;
    v = (v * 102) / 12;
    update_digital(chan, on);
    update_dac(chan, v);
  }
}

void nrpn_callback(uint8_t chan, uint16_t address, uint16_t value) {
  if (chan != 15 || address > 7)
    return;
  update_dac(address, value >> 4);
}

void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val) {
  if (chan == 0) {
    if (num < 8) {
      update_dac(num, ((uint16_t)val) << 3); //7 bit, so.. 
      //midi_send_cc(device, chan + 1, val, num);
    } else if (num < 16) {
      update_digital(num - 8, val > 0);
    }
  } else if (chan == 15) { 
    static uint8_t nrpn_state = 0;
    static uint16_t nrpn_address = 0;
    static uint16_t nrpn_value = 0;

    //NRPN
    switch (num) {
      case 99:
        //ADDR MSB
        nrpn_address = (uint16_t)val << 7;
        nrpn_state = 1;
        break;
      case 98:
        //ADDR LSB
        if (nrpn_state != 1)
          return;
        nrpn_address |= val;
        nrpn_state++;
        break;
      case 6:
        //VAL MSB
        if (nrpn_state != 2)
          return;
        nrpn_state++;
        nrpn_value = (uint16_t)val << 7;
        //XXX wait for LSB or just go for it?
        nrpn_callback(chan, nrpn_address, nrpn_value);
        break;
      case 38:
        //VAL LSB
        if (nrpn_state != 3)
          return;
        nrpn_state++;
        nrpn_value |= val;
        nrpn_callback(chan, nrpn_address, nrpn_value);
        break;
      default:
        nrpn_state = 0;
        break;
    }
  }
}

void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(true, chan, note, vel);
}

void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(false, chan, note, vel);
}

void sysex_callback(MidiDevice * device, uint16_t count, uint8_t byte0, uint8_t byte1, uint8_t byte2) {
  bool valid = false;
  if (count >= sizeof(reboot_sequence)) {
    valid = false;
    return;
  }

  //kick it off valid
  if (count == 3)
    valid = true;

  if (valid) {
    uint8_t bytes[3];
    uint8_t start_index = 3 * ((count - 1) / 3);
    uint8_t i;
    bytes[0] = byte0;
    bytes[1] = byte1;
    bytes[2] = byte2;
    for (i = start_index; i < count; i++) {
      if (bytes[i % 3] != reboot_sequence[i]) {
        valid = false;
        break;
      }
    }

    if (valid && count == sizeof(reboot_sequence))
      jump_to_bootloader();
  }
}

