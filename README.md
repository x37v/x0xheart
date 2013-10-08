xnor usb-octomod implementation
=====

So far its just the normal octobmod but controlled via MIDI

mapping:
CC on channel 1
	num 0..7 value << 3
  num 8..15 digital outs [0..7] off if zero, on otherwise
NRPN channel 16
	num 0..7 value[14bit] >> 4
Note channel 1-8
	cvout[chan] = (num * 102) / 12
  trigout[chan] = noteon

the board says OUT5,6,7,8,1,2,3,4 but they map to 0,1,2,3,4,5,6,7 in CC

usb-octobmod
=====

Originally by Greg Surges

https://bitbucket.org/pucktronix/usb-octomod/wiki/Home
