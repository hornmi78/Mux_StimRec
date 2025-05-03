# Mux_StimRec
Code directory for a multichannel stimulator/recording device. 

I'm checking if this will make it to GitHub

v-1.0, Currently working with an nucleo-l432kc (STM32L432KC). I have the 3.3V-GND, of the micro, powering the circuit. SPI communication is being used to control an external 4-channel DAC (AD5624RBRMZ-3). Pin A6 is the data line, A4 is the clock, and A3 is the SYNC line, controlled by a GPIO pin toggle. The L432KC does not have the option to use NSS Hardware control to toggle the SYNC pin low when a SPI command is sent to the DAC because it is not possible to set a data size of 24-bit which is what the DAC needs. Instead three, 8-bit SPI commans are sent. The NUCLEO-G431KB has been ordered to allow for a data size of 24-bit to be sent to the DAC in one command. Another branch will be made to explore this option. Currently a sine wave can be produced on one channel up to around 225Hz with 128 points along a single cycle. The waveform needs to have a low-pass filter to smooth the waveform but it will need to be set high enough to allow for high frequency waveforms like pulse stimulation to be used. 

