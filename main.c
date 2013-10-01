#include "midi_usb.h" //see the lufa_midi implementation

//forward declarations of callbacks
void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val);
void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);
void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel);

int main(void) {
   MidiDevice midi_device;

   //setup the device
   midi_usb_init(&midi_device);

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

void note_callback(bool on, uint8_t chan, uint8_t note, uint8_t vel) {
}

void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val) {
}

void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(true, chan, note, vel);
}

void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t vel) {
  note_callback(false, chan, note, vel);
}

