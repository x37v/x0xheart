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


/*
 * teensy 2.0

 SPI
   B0 -> SS   [DAC CS]
   B1 -> SCLK [DAC]
   B2 -> MOSI [DAC]
   B3 -> MISO [UNUSED]
   //TIE LDAC low to transfer on rising edge of CS
 MIDI
   D2 -> RXD1
   D3 -> TXD1

*/

#define DAC0_DDR DDRB
#define DAC0_ENABLE_PORT PORTB
#define DAC0_ENABLE_PIN PINB0

/*
#define DAC1_DDR DDRD
#define DAC1_ENABLE_PORT PORTD
#define DAC1_ENABLE_PIN PORTD0
*/

int main(void) {
  clock_prescale_set(clock_div_4);
  MidiDevice midi_device;

  //spi enables as outputs
  DAC0_DDR |= _BV(DAC0_ENABLE_PIN);

  /*
  //digital out, d3 [oops], c6
  DDRC |= _BV(PINC6);
  DDRD |= _BV(PIND3);
  */

  dac_sel(false, 0);

  //setup the device
  midi_usb_init(&midi_device);
  setup_spi(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK2);

  //register callbacks
  //XXX midi_register_cc_callback(&midi_device, cc_callback);
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

/*
bit 15 A/B: DAC A or DAC B Select bit
  1 = Write to DAC B
  0 = Write to DAC A
bit 14 BUF: V REF Input Buffer Control bit
  1 =  Buffered
  0 =  Unbuffered
bit 13 GA: Output Gain Select bit
  1 = 1x (V OUT = V REF * D/4096)
  0 = 2x (V OUT = 2 * V REF * D/4096)
bit 12 SHDN: Output Power Down Control bit
  1 = Output Power Down Control bit
  0 = Output buffer disabled, Output is high impedance
bit 11-0 DATA
*/

//dac a, buffered, normal gain, out buffer enabled
//0b0110 -> 0x60
#define DAC_CONTROL_NIBBLE 0x60

void update_dac(uint8_t addr, uint16_t value) {
  uint8_t out[2];

  //cut up value
  out[0] = 0x0F & (value >> 8);
  out[1] = (0xFF & value);

  //add control data
  out[0] |= DAC_CONTROL_NIBBLE;

  const uint8_t dac = 0;
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
  setp(DAC0_ENABLE_PORT, DAC0_ENABLE_PIN, !sel);
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
      uint16_t largeval = ((uint16_t)val) << 3;
      if (val & 0x1)
        largeval |= 0x7;
      update_dac(num, largeval); //7 bit, so.. 
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

