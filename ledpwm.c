/* LEDC (LED Controller) basic example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)
//
//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pitches.h"
#include "notes.h"
#include <math.h>
#include "driver/mcpwm.h"
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (4) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095 8192/9 = 819 10%
//#define LEDC_FREQUENCY          (262) // Frequency in Hertz. Set frequency at 5 kHz
#define ESC_GPIO (4)
int led_duty = 819;
#define BPM 99//116
typedef struct sheet{
  int note;
  int octive;
  float frac;
}Sheet;
typedef struct note{
  int freq_hz;
  int delay;
}Note;
int outputFrequency(int halfsteps){
  float a_term = pow(1.05946, (float)halfsteps);
  int out = (int)(F0 *a_term);
  return out;
}
Note conv(int note,int octive,float fraction_comm){
  Note output;
  output.freq_hz = 0;
  output.delay = 0;
  int noteNumber = (12 * octive)+note;
  int halfsteps_away = (4*12)+A;
  output.freq_hz =outputFrequency(noteNumber-halfsteps_away);
  if (note ==R ) output.freq_hz = 10;
  output.delay = (60000/BPM * 4 )*fraction_comm;


  return output;
}

 //project code

void playNote(Note n){


  // mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, n.freq_hz);
  ledc_set_freq(LEDC_MODE, LEDC_TIMER, n.freq_hz);
  // ledc_bind_channel_timer(ledc_mode_tspeed_mode, ledc_channel_tchannel, ledc_timer_ttimer_sel)
  int freq = ledc_get_freq(LEDC_MODE, LEDC_TIMER);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led_duty));
  // Update duty to apply the new value
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

  vTaskDelay(n.delay / portTICK_PERIOD_MS);
}
void noteTask(){
  Sheet music[] = {{ .note = A , .octive = 4 , .frac = QRT },
             {.note = A ,.octive = 3 ,.frac = QRT },
             {.note = A ,.octive = 3 ,.frac = QRT },
             {.note = A ,.octive = 3 ,.frac = QRT },
             {.note = F ,.octive = 2 ,.frac = ETH },
             {.note = C ,.octive = 3 ,.frac = ETH },

             {.note = A ,.octive = 2 ,.frac = QRT },
              {.note = F ,.octive = 3 ,.frac = ETH },
              {.note = C , .octive = 3 , .frac = ETH },
              {.note = A,.octive = 3,.frac = HLF },


              {.note = E ,.octive = 4 ,.frac = QRT },
              {.note = F ,.octive = 4 ,.frac = QRT },
              {.note = A ,.octive = 4 ,.frac = QRT },
             {.note = B ,.octive = 4 ,.frac = QRT },
             {.note = C ,.octive = 4 ,.frac = QRT },
             {.note = D ,.octive = 4 ,.frac = QRT },
             {.note = E ,.octive = 4 ,.frac = QRT },
             {.note = F ,.octive = 4 ,.frac = QRT },
             {.note = A ,.octive = 4 ,.frac = QRT },
              {.note = B ,.octive = 4 ,.frac = QRT },
              {.note = C ,.octive = 4 ,.frac = QRT },
              {.note = D ,.octive = 4 ,.frac = QRT },
              {.note = E ,.octive = 4 ,.frac = QRT },
              {.note = F ,.octive = 4 ,.frac = QRT }
            };
            //   A3, A3, A3, F2, C3,
            //   A3, F2, C3, A3,
            // 4, 4, 4, 8, 8,
         //   4, 8, 8, 2,
  int number_of_notes = 9;
  // intialize pwm
  while(1){
    for(int i=0; i<number_of_notes;i++)
    {
      Note Play = conv(music[i].note, music[i].octive, music[i].frac);
      Note pause =conv(R,0,BRK);
      playNote(Play);
      playNote(pause);
    }
  }
}
void example_ledc_init()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 2000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 50%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
void initPWM()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC_GPIO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

}
void app_main(void)
{
  // initPWM();
  example_ledc_init();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  xTaskCreate(noteTask, "alarm", 4096, NULL, configMAX_PRIORITIES, NULL);
}
// #include <stdio.h>
// #include "driver/ledc.h"
// #include "esp_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "pitches.h"
//
// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (4) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095 8192/9 = 819 10%
// //#define LEDC_FREQUENCY          (262) // Frequency in Hertz. Set frequency at 5 kHz
//
// //setup function for led run once
// void example_ledc_init(int LEDC_FREQUENCY)
// {
//     // Prepare and then apply the LEDC PWM timer configuration
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_MODE,
//         .timer_num        = LEDC_TIMER,
//         .duty_resolution  = LEDC_DUTY_RES,
//         .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
//
//     // Prepare and then apply the LEDC PWM channel configuration
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode     = LEDC_MODE,
//         .channel        = LEDC_CHANNEL,
//         .timer_sel      = LEDC_TIMER,
//         .intr_type      = LEDC_INTR_DISABLE,
//         .gpio_num       = LEDC_OUTPUT_IO,
//         .duty           = 0, // Set duty to 50%
//         .hpoint         = 0
//     };
//     ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
// }
//
//
//  //project code
//
//
//
// void app_main(void)
// {
//  int melody[] = {
//   A3, A3, A3, F2, C3,
//   A3, F2, C3, A3,
//   E4, E4, E4, F4, C4,
//   GS3,  F2, C3, A3,
//   A4, A3, A3, A4, GS4, G4, FS4, F4, FS4, AS3, DS4, D4, CS3,
//   C4, B3, C4, F2, GS3, F2, GS3, C4, A3, C4, E4
// };
//
// // note durations: 4 = quarter note, 8 = eighth note, etc.:
// double noteDurations[] = {
//   4, 4, 4, 8, 8,
//   4, 8, 8, 2,
//   4, 4, 4, 8, 8,
//   4, 8, 8, 2,
//   4, 8, 8, 4, 8, 8, 8, 8, 8, 8, 4, 8, 8,
//   8, 8, 8, 8, 4, 8, 8, 4, 8, 8, 2
// };
//
//
//    //double led_duty = 0;
//
//    while(true){
//
//
//     // Set the LEDC peripheral configuration
//     //printf("LED Duty Percent: %.3f \n",  (led_duty/8192)*100);
//     for (int thisNote = 0; thisNote < 42; thisNote++) {
//
//     // to calculate the note duration, take one second divided by the note type.
//     //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
//     int noteDuration = 1000 / noteDurations[thisNote];
//
//     example_ledc_init(melody[thisNote]);
//     ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, led_duty));
//     // Update duty to apply the new value
//      ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//
//     vTaskDelay(noteDuration / portTICK_RATE_MS);
//
//     int pauseBetweenNotes = noteDuration * 2.3;
//     example_ledc_init(1);
//     vTaskDelay(pauseBetweenNotes / portTICK_RATE_MS);
//
//
//
//     // to distinguish the notes, set a minimum time between them.
//     // the note's duration + 30% seems to work well:
//
//     //delay(pauseBetweenNotes);
//     // stop the tone playing:
//     //noTone(8);
//   }
//
//     vTaskDelay(10 / portTICK_RATE_MS);
//
//    }
// }
