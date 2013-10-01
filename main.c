#include "midi_usb.h" //see the lufa_midi implementation
#include "spi.h"
#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>

//forward declarations of callbacks
void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val);
void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);
void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);

void update_dac(uint8_t addr, uint16_t value);
void dac_sel(bool sel, uint8_t dac);

#define DAC0_DDR DDRB
#define DAC0_ENABLE_PORT PORTB
#define DAC0_ENABLE_PIN PINB0

#define DAC1_DDR DDRD
#define DAC1_ENABLE_PORT PORTD
#define DAC1_ENABLE_PIN PORTD0

int main(void) {
  int i = 0;
  clock_prescale_set(clock_div_4);
  MidiDevice midi_device;

  //spi enables as outputs
  DAC0_DDR |= _BV(DAC0_ENABLE_PIN);
  DAC1_DDR |= _BV(DAC1_ENABLE_PIN);

  dac_sel(false, 0);
  dac_sel(false, 1);

  //setup the device
  midi_usb_init(&midi_device);
  setup_spi(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK2);

  //register callbacks
  midi_register_cc_callback(&midi_device, cc_callback);
  midi_register_noteon_callback(&midi_device, note_on_callback);
  midi_register_noteoff_callback(&midi_device, note_off_callback);

  while(1){
    //processes input from the midi device
    //and calls the appropriate callbacks
    midi_device_process(&midi_device);
  }

  return 0; //never happens
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

void dac_sel(bool sel, uint8_t dac) {
  if (dac == 0) {
    if (sel)
      DAC0_ENABLE_PORT &= ~_BV(DAC0_ENABLE_PIN);
    else
      DAC0_ENABLE_PORT |= _BV(DAC0_ENABLE_PIN);
  } else {
    if (sel)
      DAC1_ENABLE_PORT &= ~_BV(DAC1_ENABLE_PIN);
    else
      DAC1_ENABLE_PORT |= _BV(DAC1_ENABLE_PIN);
  }
}

void note_callback(bool on, uint8_t chan, uint8_t note, uint8_t vel) {
  //XXX implement
}

void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val) {
  if (chan == 0) {
    if (num < 8) {
      update_dac(num, ((uint16_t)val) << 3); //7 bit, so.. 
      //midi_send_cc(device, chan + 1, val, num);
    }
  }
}

void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(true, chan, note, vel);
}

void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(false, chan, note, vel);
}

