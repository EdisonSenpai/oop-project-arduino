#include <FastLED.h>

// Sound definitions

#define OCTAVE    1     // Group buckets into octaves (use the long output function LOG_OUT 1)
#define OCT_NORM  0     // Don't normalise octave intensities by number of bins
#define FHT_N     256   // Set to 256 points FHT

#include <FHT.h>        // The library needs to be included here to compile

// Led_strip definitions

#define LED_PIN     2
#define LED_TYPE    WS2812
#define COLOR_ORDER RGB

// Led_strip class for pin

class LedStripPin {

protected:

  int ledPin;

public:

  LedStripPin();
  LedStripPin(int);

  int getStripPin();

};

LedStripPin::LedStripPin() : ledPin(0) {}
LedStripPin::LedStripPin(int lP) : ledPin(lP) {}

int LedStripPin::getStripPin() {return ledPin;}

// Params for width & height

const uint8_t kMatrixWidth = 7;
const uint8_t kMatrixHeight = 8;

#define NUM_LEDS (kMatrixWidth * kMatrixHeight - 5)
//#define NUM_LEDS 51

CRGB leds[NUM_LEDS];

int counter2 = 0;

//int noise[] = {204,198,100,85,85,80,80,80};
//int noise[] = {204,188,68,73,150,98,88,68};               // noise level determined by playing pink noise and seeing levels [trial and error]{204,188,68,73,150,98,88,68}
//int noise[] = {204,190,108,85,65,65,55,60};               // noise for Mega adk
int noise[] = {204,195,100,90,85,80,75,75};                 // noise for NANO
float noise_fact[] = {15, 7, 1.5, 1, 1.2, 1.4, 1.7, 3};      // noise level determined by playing pink noise and seeing levels [trial and error]{204,188,68,73,150,98,88,68}
float noise_fact_adj[] = {15, 7, 1.5, 1, 1.2, 1.4, 1.7, 3};  // noise level determined by playing pink noise and seeing levels [trial and error]{204,188,68,73,150,98,88,68}

void setup() {

  Serial.begin(115200);
  delay(1000);

  //LedStripPin ledPIN(LED_PIN);
  //const int led_pin = ledPIN.getStripPin();
  
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  //TIMSK = 0;    // Turn off timer0 for lower jitter
  
  ADCSRA = 0xe5;  // Set the adc to free running mode
  ADMUX = 0x40;   // Use adc0
  DIDR0 = 0x01;   // Turn off the digital input for adc0

}

void loop() {

  int prev_j[8];
  int prev_oct_j;
  int beat = 0;
  int prev_beat = 0;
  int saturation = 0;
  int saturation_prev = 0;
  int brightness = 0;
  int brightness_prev = 0;
  int counter = 0;
  int led_index = 0;

  while (1) {   //reduces jitter

    cli();  // UDRE interrupt slows this way down on arduino 1.0

    for (int i = 0; i < FHT_N; i++) {    // Save 256 samples

      while (!(ADCSRA & 0x10));   // Wait for adc to be ready

      ADCSRA = 0xf5;              // Restart adc
      byte m = ADCL;              // Fetch adc data
      byte j = ADCH;

      int k = (j << 8) | m;       // Form into an int
      k -= 0x0200;                // Form into a signed int
      k <<= 6;                    // Form into a 16b signed int

      fht_input[i] = k;           // Put reak data into bins 

    }

    fht_window();                 // Window the data for better frequency response
    fht_reorder();                // Reorder the data before doing fht
    fht_run();                    // Process the data in the fht
    fht_mag_octave();             // Take the output of the fht: fht_mag_log()

    // Every 50th loop, adjust the volume accourding to the value on A2 (Pot)

    if (counter >= 50) {

      ADMUX = 0x40 | (1 & 0x07);                    // Set admux to look at analog pin A1 - Master Volume

      while (!(ADCSRA & 0x10));                     // Wait for adc to be ready
      
      ADCSRA = 0xf5;                                 // Restart adc
      delay(10);

      while (!(ADCSRA & 0x10));                     // Wait for adc to be ready

      ADCSRA = 0xf5;                                // Restart adc
      byte m = ADCL;                                // Fetch adc data
      byte j = ADCH;

      int k = (j << 8) | m;                         // Form into an int
      float master_volume = (k + 0.1) / 100 + .5;   // So the value will be between ~0.5 and 1.5

      //Serial.println(master_volume);

      for (int i = 1; i < 8; i++) {

        noise_fact_adj[i] = noise_fact[i] * master_volume;

      }

      ADMUX = 0x40 | (0 & 0x07);                    // Set admux back to look at A0 analog pin (to read the microphone input)

      counter = 0;

    }

    sei();

    counter++;

    // End of Fourier Transform code - output is stored in fht_oct_out[i]

    // i = 0 - 7 frequency (octave) bins (don't use 0 or 1)
    // fht_oct_out[1] = amplitude of frequency for bin 1

    /* for loop: 
        a) removes background noise average and takes absolute value 
        b) low / high pass filter as still very noisy
        c) maps amplitude of octave to a colour between blue and red 
        d) sets pixel colour to amplitude of each frequency (octave)
    */

    for (int i = 1; i < 8; i++) {                 // Goes through each octave, skip the first 1, which is not useful

      int j;

      j = (fht_oct_out[i] - noise[i]);             // Take the pink noise average level out, take the absolute value to avoid negative numbers

      if (j < 10) {

        j = 0;

      }
      else {

        j *= noise_fact_adj[i];

        if (i > 180) {

          if (i >= 7) {
            
            beat += 2;

          }            
          else {
            
            beat++;  

          }        

        }

        j /= 30;
        j *= 30;  // Force it to more discrete values

      }

      prev_j[i] = j;

      //Serial.print(j);
      //Serial.print(" ");

      // This fills in 11 LED's with interpolated values between each of the 8 OCT values

      if (i >= 2) {

        led_index = 2 * i - 3;
        prev_oct_j = (j + prev_j[i-1]) / 2;

        saturation = constrain(j + 30, 0, 255);
        saturation_prev = constrain(prev_oct_j + 30, 0, 255);

        brightness = constrain(j, 0, 255);
        brightness_prev = constrain(prev_oct_j, 0, 255);

        if (brightness == 255) {

          saturation = 50;
          brightness = 200;

        }
                
        if (brightness_prev == 255) {

          saturation_prev = 50;
          brightness_prev = 200;

        }     

        for (uint8_t y = 0; y < kMatrixHeight; y++) {

          leds[XY(led_index - 1, y)] = CHSV(j + y * 30, saturation, brightness);

          if (i > 2) {

            prev_oct_j = (j + prev_j[i-1]) / 2;
            leds[XY(led_index - 2, y)] = CHSV(prev_oct_j + y * 30, saturation_prev, brightness_prev);

          }

        }                           

      }

    }

    if (beat >= 7) {

      fill_solid(leds, NUM_LEDS, CRGB::Gray);
      FastLED.setBrightness(120);
      //FastLED.setBrightness(200);
      
    }
    else {

      if (prev_beat != beat) {

        FastLED.setBrightness(4- + 5 * beat * beat);
        prev_beat = beat;

      }

    }
    
    FastLED.show();

    if (beat) {

      counter2 += ((beat + 4) / 2 - 2);

      if (counter2 < 0) {

        counter2 = 1000;

      }
      
      if (beat > 3 && beat < 7) {

        FastLED.delay(30);

      }
      
      beat = 0;      

    }

    //Serial.println();

  }

}

// Params for different pixel layouts

const bool kMatrixSerpentineLayout = false;

// Set 'kMatrixSerpentineLayout' to false if your pixels are laid out all running the same way
// Set 'kMatrixSerpentineLayout' to true if your pixels are laid out back-and-forth

uint16_t XY(uint8_t x, uint8_t y) {

  uint16_t i;

  if (kMatrixSerpentineLayout == false) {
    
    i = (y * kMatrixWidth) + x;
  
  }

  if (kMatrixSerpentineLayout == true) {
    
    if (y & 0x01) {

      // Odd rows run backwards

      uint8_t reverseX = (kMatrixWidth - 1) - x;
      i = (y * kMatrixWidth) + reverseX;

    }
    else {

      // Even rows run forwards

      i = (y * kMatrixWidth) + x;      
      
    }

  }
  
  i = (i + counter2) % NUM_LEDS;

  return i;

}