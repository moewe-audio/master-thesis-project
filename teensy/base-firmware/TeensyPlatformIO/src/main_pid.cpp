#include <Arduino.h>
#include <i2c_device.h>
#include "max98389.h"
#include <Audio.h>
#include <string.h>
#include <limits.h>

#include "au_Biquad.h"
#include "au_config.h"

#include "TransducerFeedbackCancellation.h"
#include "ForceSensing.h"
#include "TeensyEeprom.h"
#include "TeensySerialCommands.h"
#include "MidiComms.h"
#include "pid.h"
#include "filter3rdO.h"

#define teensy_sample_t int16_t

//Board revision definitions (only define one):
// #define BOARD_VERSION_REV_A
#define BOARD_VERSION_REV_B

#define BUILD_RELEASE 1 //Set to 1 when generating a release build .hex file

// Write the defined serial number byte to EEPROM when flashing if enabled
// Once done, disable the write serial  to EEPROM and reflash Teensy (avoids the code writing the serial number at every startup).
#define WRITE_SERIAL_NUMBER_TO_FLASH 0
#if WRITE_SERIAL_NUMBER_TO_FLASH
#define TEENSY_SERIAL_NUMBER 9
#endif

//If enabled, this initialises the parameters stored in the EEPROM to their default values.
//Once done, this option should be disabled, the firmware recompiled and reflashed to avoid writing to the EEPROM at every startup (overwriting the existing values).
#define INITIALISE_EEPROM_VALUES 0

#define RESONANT_FREQ_HZ 89.0

#define MAX_SERIAL_INPUT_CHARS 256

#if BUILD_RELEASE
static const unsigned int VERSION_MAJ = 1;
static const unsigned int VERSION_MIN = 3;
#else
//Set version number to 255.255 for debug builds to avoid confusion
static const unsigned int VERSION_MAJ = 255;
static const unsigned int VERSION_MIN = 255;
#endif

const char VERSION_NOTES[] = "Added MIDI control of resonant frequency and tone level.";


// Import generated code here to view block diagram https://www.pjrc.com/teensy/gui/
// GUItool: begin automatically generated code
AudioInputI2SQuad        i2s_quad_in;
AudioInputUSB            usb_in;

AudioOutputI2SQuad      i2s_quad_out;
AudioOutputUSB           usb_out;

AudioRecordQueue         queue_inR_usb;         //xy=487,179
AudioRecordQueue         queue_inL_usb;         //xy=488,146
AudioRecordQueue         queue_inL_i2s;         //xy=490,338
AudioRecordQueue         queue_inR_i2s;         //xy=492,414

AudioPlayQueue           queue_outR_i2s;         //xy=653,182
AudioPlayQueue           queue_outL_i2s;         //xy=654,147
AudioPlayQueue           queue_outR_usb;         //xy=660,410
AudioPlayQueue           queue_outL_usb;         //xy=664,339

AudioConnection          patchCord1(i2s_quad_in, 2, queue_inL_i2s, 0);
AudioConnection          patchCord2(i2s_quad_in, 3, queue_inR_i2s, 0);
AudioConnection          patchCord3(usb_in, 0, queue_inL_usb, 0);
AudioConnection          patchCord4(usb_in, 1, queue_inR_usb, 0);

#ifdef BOARD_VERSION_REV_B
AudioConnection          patchCord5(queue_outR_i2s, 0, i2s_quad_out, 3);
AudioConnection          patchCord6(queue_outL_i2s, 0, i2s_quad_out, 2);
#endif

#ifdef BOARD_VERSION_REV_A
AudioConnection          patchCord5(queue_outR_i2s, 0, i2s_quad_out, 1);
AudioConnection          patchCord6(queue_outL_i2s, 0, i2s_quad_out, 0);
#endif


AudioConnection          patchCord7(queue_outR_usb, 0, usb_out, 1);
AudioConnection          patchCord8(queue_outL_usb, 0, usb_out, 0);

AudioControlSGTL5000     sgtl5000_1;     //xy=527,521
// GUItool: end automatically generated code

IntervalTimer led_blink_timer;

bool configured = false;

char serial_input_buffer[MAX_SERIAL_INPUT_CHARS] = {0};
int input_char_index = 0;

TransducerFeedbackCancellation transducer_processing;
TransducerFeedbackCancellation::Setup current_cancellation_setup;

ForceSensing force_sensing;
Biquad meter_filter;
TeensyEeprom teensy_eeprom;
uint8_t serial_number;

const float PP = 0.f;
const float PD = 1.f;
const float PDD = 0.0f;
const float dt = 1.0f / AUDIO_SAMPLE_RATE_EXACT;

PIDController pid(PP, PD, PDD, dt);

std::vector<ThirdOrderFilter> feedbackFilters;

float outputGain = 1.0f;

//Basic error states that can occur, used for debug prints and LED blink interval.
enum class ErrorStates
{
    NORMAL_OPERATION,
    AMP_NOT_CONFIGURED,
    PLAY_BUFFER_ERROR,
    DEBUG,
    OTHER
};

ErrorStates current_error_state;
TeensyEeprom::BoardRevision board_revision;

static const unsigned long LED_BLINK_INTERVAL_NORMAL_OPERATION              =  1000000; //1s
static const unsigned long LED_BLINK_INTERVAL_AMP_NOT_CONFIGURED            =   500000; //500ms
static const unsigned long LED_BLINK_INTERVAL_PLAY_BUFFER_ERROR             =   250000; //250ms
static const unsigned long LED_BLINK_INTERVAL_DEBUG                         =   100000; //100ms
static const unsigned long LED_BLINK_INTERVAL_OTHER                         = 10000000; //10s


void setErrorState(ErrorStates error_state);
void printCurrentTime();
void processSerialInput(char new_char);
void blinkLED();
void readAndApplyEepromParameters();
void writeEepromParameters();
void resetToDefaultParameters();
void sendSerialDetails();

void setResonantFrequency(sample_t resonant_frequency_hz);
void setToneLevel(sample_t tone_level_db);
void setResonanceQ(sample_t resonance_q);
void setWidebandGain(sample_t wideband_gain_db);
void setResonanceGain(sample_t resonance_gain_db);
void setOutputLowpassFc(sample_t cutoff_frequency_hz);
void setInputLowpassFc(sample_t cutoff_frequency_hz);


// MIDI-related functions
void rxPitchChange(uint8_t channel, int pitch);
void rxProgrammeChange(uint8_t channel, uint8_t programme);
void rxControlChange(uint8_t channel, uint8_t control_number, uint8_t control_value);
void txForceSenseVal(sample_t force_sense_val);

//To reduce latency, set MAX_BUFFERS = 8 in play_queue.h and max_buffers = 8 in record_queue.h

void setup() {

#if WRITE_SERIAL_NUMBER_TO_FLASH
    teensy_eeprom.write(TeensyEeprom::ByteParameters::SERIAL_NUMBER, TEENSY_SERIAL_NUMBER);
#endif

    serial_number = teensy_eeprom.read(TeensyEeprom::ByteParameters::SERIAL_NUMBER);

    // set up Teensy's built in LED
    pinMode(LED_BUILTIN, OUTPUT);
    led_blink_timer.begin(blinkLED, LED_BLINK_INTERVAL_NORMAL_OPERATION);  

    //Setup pin 2 to input due to hardware mod (pin 32 is i2s output and connected to pin 2). DO NOT USE PIN 2 AS AN OUTPUT!!
    pinMode(2, INPUT);

    printf("Teensy has booted.\r\n");
    sendSerialDetails();

    //Configure amp IC over i2c
    max98389 max;
    max.begin(400 * 1000U);
    // Check that we can see the sensor and configure it.
    configured = max.configure();
    if (configured) {
        Serial.println("Amplifer chip successfully configured");
    } else {
        Serial.println("Error! Amplifier chip not successfully configured.");
    }

    AudioMemory(512);
    sgtl5000_1.enable();
    sgtl5000_1.volume(1);

    // force_sensing.setup();

    //Setup feedback cancellation
    resetToDefaultParameters();

#if INITIALISE_EEPROM_VALUES
    writeEepromParameters();
#endif
    
    readAndApplyEepromParameters();

    printf("Resonant frequency: %f\r\n",current_cancellation_setup.resonant_frequency_hz);

    usbMIDI.setHandlePitchChange(rxPitchChange);
    usbMIDI.setHandleProgramChange(rxProgrammeChange);
    usbMIDI.setHandleControlChange(rxControlChange);

    //Begin audio buffer queues
    queue_inL_usb.begin();
    queue_inR_usb.begin();
    queue_inL_i2s.begin();
    queue_inR_i2s.begin();

    // feedbackFilter.setCoefficients(-1.998184, 0.99843053, 0.00078473, 0.f, -0.00078473); // Bandpass 120hz, Q: 7
    // feedbackFilter2.setCoefficients(-1.99425979, 0.99533083, 0.00233458, 0.f, -0.00233458); // Bandpass 230hz, Q: 7
    // feedbackFilter.setCoefficients(-1.9939623811601523, 0.9951284359939072, 0.0024357820030463935, 0.0, -0.0024357820030463935);
    for (int i = 0; i < 2; i++)
    {
        feedbackFilters.push_back(ThirdOrderFilter(AUDIO_SAMPLE_RATE_EXACT));
        feedbackFilters[i].setFilterParams(220.3f, 10.f);
    }
    // feedbackFilter3.setCoefficients(-1.9831627883306575, 0.9841730120573678, 0.007913493971316065, 0.0, -0.007913493971316065); // BP 225, Q: 2
    // feedbackFilter3.setCoefficients(-1.99453865, 0.99552316, 0.00223842, 0.f, -0.00223842); // Bandpass 240hz, Q: 7
    // feedbackFilter4.setFilterParams(224.f, 2.f);
    // feedbackFilter4.setCoefficients(-1.99453865, 0.99552316, 0.00223842, 0.f, -0.00223842); // Bandpass 200hz, Q: 7
    // feedbackFilter2.setCoefficients(-1.9841629, 0.98506235, 0.00746882, 0.f,  -0.00746882); // Bandpass 230hz, Q: 2
    // feedbackFilter.setCoefficients(-1.98275923, 0.98304904, 0.99237207, -1.98275923, 0.99067697); // Notch 120hz
}

teensy_sample_t buf_inL_usb[AUDIO_BLOCK_SAMPLES];
teensy_sample_t buf_inR_usb[AUDIO_BLOCK_SAMPLES];
teensy_sample_t buf_inL_i2s[AUDIO_BLOCK_SAMPLES];
teensy_sample_t buf_inR_i2s[AUDIO_BLOCK_SAMPLES];
unsigned long total_sample_count = 0;

void loop() {

    int16_t *bp_outL_usb, *bp_outR_usb, *bp_outL_i2s, *bp_outR_i2s;

    // Wait for i2s (amp) channels to have content
    while (!queue_inL_i2s.available() || !queue_inR_i2s.available());

    memcpy(buf_inL_i2s, queue_inL_i2s.readBuffer(), sizeof(teensy_sample_t)*AUDIO_BLOCK_SAMPLES);
    memcpy(buf_inR_i2s, queue_inR_i2s.readBuffer(), sizeof(teensy_sample_t)*AUDIO_BLOCK_SAMPLES);
    
    //Free queue input buffers
    queue_inL_i2s.freeBuffer();
    queue_inR_i2s.freeBuffer();

    // Get pointers to "empty" output buffers
    bp_outL_i2s = queue_outL_i2s.getBuffer();
    bp_outR_i2s = queue_outR_i2s.getBuffer();
    bp_outL_usb = queue_outL_usb.getBuffer();
    bp_outR_usb = queue_outR_usb.getBuffer();

    //Loop through each sample in the buffers
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {   

        //Convert all incoming samples from int16 to normalised float
        // sample_t usb_in_l = intToNormalised<teensy_sample_t>(buf_inL_usb[i]);
        // sample_t usb_in_r = intToNormalised<teensy_sample_t>(buf_inR_usb[i]);
        sample_t amp_in_voltage = intToNormalised<teensy_sample_t>(buf_inL_i2s[i]);
        sample_t amp_in_current = intToNormalised<teensy_sample_t>(buf_inR_i2s[i]);

        sample_t usb_out_l, usb_out_r, amp_out;
        // sample_t out  = pid.processSample(amp_in_current);
        sample_t out = 0.f;
        for (int j = 0; j < feedbackFilters.size(); j++)
        {
            out += feedbackFilters[j].process(amp_in_current);
        }
        out *= 0.25f;
        usb_out_l = amp_in_current;
        usb_out_r = out * outputGain;
        amp_out = out * outputGain;

        // Convert from normalised float back to int16 and add into output buffers
        bp_outL_i2s[i] = normalisedToInt<teensy_sample_t>(amp_out);
        bp_outR_i2s[i] = normalisedToInt<teensy_sample_t>(amp_out);
        bp_outL_usb[i] = normalisedToInt<teensy_sample_t>(usb_out_l);
        bp_outR_usb[i] = normalisedToInt<teensy_sample_t>(usb_out_r);

        total_sample_count++;
    }

    // Play output buffers. Retry until success.
    while(queue_outL_i2s.playBuffer()){
        Serial.println("Play i2s left fail.");
    }
    while(queue_outR_i2s.playBuffer()){
        Serial.println("Play i2s right fail.");
    }
    while(queue_outL_usb.playBuffer()){
        Serial.println("Play usb left fail.");
    }
    while(queue_outR_usb.playBuffer()){
        Serial.println("Play usb right fail.");
    }

    //Process USB Serial input (debugging)
    while (Serial.available()) {
        char new_char = Serial.read();
        processSerialInput(new_char);
    }

    //Process USB MIDI input (configuring parameters)
    usbMIDI.read();
}

void setErrorState(ErrorStates error_state)
{
    current_error_state = error_state;
    printCurrentTime();
    switch (error_state)
    {
    case ErrorStates::NORMAL_OPERATION:
        printf(" Entering normal operation.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_NORMAL_OPERATION);
        break;

    case ErrorStates::AMP_NOT_CONFIGURED:
        printf(" Error occurred while attempting to configure the MAX98389 chip via i2c. Check whether Teensy is connected to the board correctly.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_AMP_NOT_CONFIGURED);
        break;
    
    case ErrorStates::PLAY_BUFFER_ERROR:
        printf(" Error occured while playing output buffer queue.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_PLAY_BUFFER_ERROR);
        break;

    case ErrorStates::OTHER:
        printf(" Other (generic) error occurred.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_OTHER);
        break;

    case ErrorStates::DEBUG:
        printf(" Entered debug mode.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_DEBUG);
        break;
    
    default:
        printf(" Unknown error occurred.\r\n");
        led_blink_timer.update(LED_BLINK_INTERVAL_OTHER);
        break;
    }
}

void printCurrentTime()
{
    int time_s = millis() / 1000;
    int hours = time_s / 3600;
    int minutes = (time_s - (hours * 3600)) / 60;
    int seconds = time_s - (hours * 3600) - (minutes * 60);  
    printf("%02d:%02d:%02d", hours, minutes, seconds);
}

void readAndApplyEepromParameters()
{
    current_cancellation_setup.resonant_frequency_hz = teensy_eeprom.read(TeensyEeprom::FloatParameters::RESONANT_FREQUENCY_HZ);
    current_cancellation_setup.resonance_peak_gain_db = teensy_eeprom.read(TeensyEeprom::FloatParameters::RESONANT_GAIN_DB);
    current_cancellation_setup.resonance_q = teensy_eeprom.read(TeensyEeprom::FloatParameters::RESONANT_Q);
    current_cancellation_setup.resonance_tone_level_db =  teensy_eeprom.read(TeensyEeprom::FloatParameters::TONE_LEVEL_DB);
    current_cancellation_setup.inductance_filter_coefficient =  teensy_eeprom.read(TeensyEeprom::FloatParameters::INDUCTANCE_FILTER_COEFFICIENT);
    current_cancellation_setup.transducer_input_wideband_gain_db =  teensy_eeprom.read(TeensyEeprom::FloatParameters::BROADBAND_GAIN_DB);
    current_cancellation_setup.sample_rate_hz = AUDIO_SAMPLE_RATE_EXACT;
    current_cancellation_setup.amplifier_type = TransducerFeedbackCancellation::AmplifierType::CURRENT_DRIVE;
    current_cancellation_setup.lowpass_transducer_io = true;
    transducer_processing.setup(current_cancellation_setup);

    force_sensing.setResonantFrequencyHz(teensy_eeprom.read(TeensyEeprom::FloatParameters::RESONANT_FREQUENCY_HZ));
    force_sensing.setWindowSizePeriods(teensy_eeprom.read(TeensyEeprom::ByteParameters::GOERTZEL_WINDOW_LENGTH));
    force_sensing.setRawDampedValue(teensy_eeprom.read(TeensyEeprom::FloatParameters::DAMPED_CALIBRATION_VALUE));
    force_sensing.setRawUndampedValue(teensy_eeprom.read(TeensyEeprom::FloatParameters::UNDAMPED_CALIBRATION_VALUE));

    uint8_t stored_maj_version = teensy_eeprom.read(TeensyEeprom::ByteParameters::LAST_SAVED_MAJ_VERSION);
    uint8_t stored_min_version = teensy_eeprom.read(TeensyEeprom::ByteParameters::LAST_SAVED_MIN_VERSION);


    printf("Read parameters from flash. Parameters were stored in V%d.%d. Resonant frequency now: %f\r\n",stored_maj_version, stored_min_version, current_cancellation_setup.resonant_frequency_hz);
    

}

void writeEepromParameters()
{
    teensy_eeprom.write(TeensyEeprom::FloatParameters::RESONANT_FREQUENCY_HZ, current_cancellation_setup.resonant_frequency_hz);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::RESONANT_GAIN_DB, current_cancellation_setup.resonance_peak_gain_db);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::RESONANT_Q, current_cancellation_setup.resonance_q);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::TONE_LEVEL_DB, current_cancellation_setup.resonance_tone_level_db);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::INDUCTANCE_FILTER_COEFFICIENT, current_cancellation_setup.inductance_filter_coefficient);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::BROADBAND_GAIN_DB, current_cancellation_setup.transducer_input_wideband_gain_db);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::DAMPED_CALIBRATION_VALUE, force_sensing.getRawDampedValue());
    teensy_eeprom.write(TeensyEeprom::FloatParameters::UNDAMPED_CALIBRATION_VALUE, force_sensing.getRawUndampedValue());
    teensy_eeprom.write(TeensyEeprom::FloatParameters::OUTPUT_LPF_CUTOFF_HZ, current_cancellation_setup.output_to_transducer_lpf_cutoff_hz);
    teensy_eeprom.write(TeensyEeprom::FloatParameters::INPUT_LPF_CUTOFF_HZ, current_cancellation_setup.input_from_transducer_lpf_cutoff_hz);

    teensy_eeprom.write(TeensyEeprom::ByteParameters::GOERTZEL_WINDOW_LENGTH, force_sensing.getWindowSizePeriods());
    teensy_eeprom.write(TeensyEeprom::ByteParameters::LAST_SAVED_MAJ_VERSION, VERSION_MAJ);
    teensy_eeprom.write(TeensyEeprom::ByteParameters::LAST_SAVED_MIN_VERSION, VERSION_MIN);
    printCurrentTime();
    printf(" Parameters saved to EEPROM.\r\n");
}



void processSerialInput(char new_char)
{
    if (input_char_index == (MAX_SERIAL_INPUT_CHARS - 1))
    {
        printf("Error! Max number of serial input characters (%d) per line exceeded. Input buffer reset.\r\n",MAX_SERIAL_INPUT_CHARS);
        input_char_index = 0;
        return;
    }

    serial_input_buffer[input_char_index++] = new_char;
    if (new_char == '\n')
    {

        char* parameter_arg = strtok(serial_input_buffer, " "); //Split input string on space
        char* value_arg1 = strtok(NULL, " "); // First argument
        char* value_arg2 = strtok(NULL, " "); // Second argument
        char* value_arg3 = strtok(NULL, " "); // Third argument

        if (!strncmp(parameter_arg, SerialCommands::kDebugModeString, strlen(SerialCommands::kDebugModeString)))
        {
            setErrorState(ErrorStates::DEBUG);
        }
        else if (!strncmp(parameter_arg, SerialCommands::kNormalModeString, strlen(SerialCommands::kNormalModeString)))
        {
            setErrorState(ErrorStates::NORMAL_OPERATION);
        }
        else if (!strncmp(parameter_arg, SerialCommands::kResetParametersString, strlen(SerialCommands::kResetParametersString)))
        {
            resetToDefaultParameters();
        }
        else if (!strncmp(parameter_arg, SerialCommands::kSaveToEepromString, strlen(SerialCommands::kSaveToEepromString)))
        {
            writeEepromParameters();
        }
        else if (!strncmp(parameter_arg, SerialCommands::kHelpString, strlen(SerialCommands::kHelpString)))
        {
            printSerialHelp();
        }
        else if (!strncmp(parameter_arg, SerialCommands::kCalibrateDamped, strlen(SerialCommands::kCalibrateDamped)))
        {
            force_sensing.calibrateDamped();
        }
        else if (!strncmp(parameter_arg, SerialCommands::kCalibrateUndamped, strlen(SerialCommands::kCalibrateUndamped)))
        {
            force_sensing.calibrateUndamped();
        }
        else if (!strncmp(parameter_arg, SerialCommands::kInfoString, strlen(SerialCommands::kInfoString)))
        {
            sendSerialDetails();
        }
        
        else
        {
            if (!strncmp(parameter_arg, SerialCommands::kResonantFreqString, strlen(SerialCommands::kResonantFreqString)))
            {
                if (value_arg1)
                {
                    setResonantFrequency(atof(value_arg1));
                    printf("Resonant frequency set to: %fHz\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.resonant_frequency_hz);
                }
            }
            if (!strncmp(parameter_arg, SerialCommands::kOutputGain, strlen(SerialCommands::kOutputGain)))
            {
                if (value_arg1)
                {
                    outputGain = atof(value_arg1);
                    printf("Output gain set to: %f\r\n", outputGain);
                }
                else
                {
                    printf("Current output gain:%f\n", outputGain);
                }
            }
            if (!strncmp(parameter_arg, SerialCommands::kFeedbackBandpass, strlen(SerialCommands::kFeedbackBandpass)))
            {
                if (value_arg1 && value_arg2 && value_arg3)
                {
                    float freq = atof(value_arg2);
                    float q = atof(value_arg3);
                    int idx = atoi(value_arg1);
                    if (idx >= feedbackFilters.size())
                    {
                        printf("Error: Invalid filter index. Max index is %i\r\n", feedbackFilters.size() - 1);
                    } else 
                    {
                        printf("Setting bandpass filter %i: Frequency = %f Hz, Q = %f\r\n", idx, freq, q);
                        feedbackFilters[idx].setFilterParams(freq, q);
                    }
                }
                else
                {
                    if (value_arg1)
                    {
                        int idx = atoi(value_arg1);                   
                        if (idx < feedbackFilters.size())
                        {
                            printf("Current parameters are: Frequency = %f Hz, Q = %f\r\n", feedbackFilters[idx].getFc(), feedbackFilters[idx].getQ());
                        }
                    }
                    printf("Error: Missing arguments. Usage: %s <idx> <frequency> <Q>\r\n", SerialCommands::kFeedbackBandpass);
                }
            }
            else if (!strncmp(parameter_arg, SerialCommands::kToneLevelString, strlen(SerialCommands::kToneLevelString)))
            {
                if (value_arg1)
                {
                    setToneLevel(atof(value_arg1));
                    printf("Tone level set to: %fdB\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.resonance_tone_level_db);
                }
            }
            else if (!strncmp(parameter_arg, SerialCommands::kResonantQString, strlen(SerialCommands::kResonantQString)))
            {
                if (value_arg1)
                {
                    setResonanceQ(atof(value_arg1));
                    printf("Resonance q set to: %f\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.resonance_q);
                }
            }

            else if (!strncmp(parameter_arg, SerialCommands::kResonantGainString, strlen(SerialCommands::kResonantGainString)))
            {
                if (value_arg1)
                {
                    setResonanceGain(atof(value_arg1));
                    printf("Resonance gain set to: %fdB\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.resonance_peak_gain_db);
                }
            }

            else if (!strncmp(parameter_arg, SerialCommands::kWidebandGainString, strlen(SerialCommands::kWidebandGainString)))
            {
                if (value_arg1)
                {
                    setWidebandGain(atof(value_arg1));
                    printf("Wideband gain set to: %fdB\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.transducer_input_wideband_gain_db);
                }
            }
            else if (!strncmp(parameter_arg, SerialCommands::kLowpassOutputFreq, strlen(SerialCommands::kLowpassOutputFreq)))
            {
                if (value_arg1)
                {
                    setOutputLowpassFc(atof(value_arg1));
                    printf("Output lowpass cutoff frequency set to: %fHz\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.output_to_transducer_lpf_cutoff_hz);
                }
            }

            else if (!strncmp(parameter_arg, SerialCommands::kLowpassInputFreq, strlen(SerialCommands::kLowpassInputFreq)))
            {
                if (value_arg1)
                {
                    setInputLowpassFc(atof(value_arg1));
                    printf("Input lowpass cutoff frequency set to: %fHz\r\n", atof(value_arg1));
                }
                else
                {
                    printf("%f\n", current_cancellation_setup.input_from_transducer_lpf_cutoff_hz);
                }
            }
           
        }

        input_char_index = 0;
    }
}

void rxPitchChange(uint8_t channel, int pitch)
{
    //Library parses MIDI channels as 1-16. Subtract 1 to make it 0-15.
    channel--;

    // printf("Pitch bend: %f on channel %d\r\n",MidiComms::pitchBendToNormalised(pitch), channel);
    switch (static_cast<MidiComms::PitchBendChannels>(channel))
    {
        case MidiComms::PitchBendChannels::RESONANT_FREQUENCY:
            setResonantFrequency(pitch - MidiComms::MIN_PITCH_BEND_VALUE);
            break;

        default:
            printf("Undefined pitch bend channel used (%d)\r\n",channel);
            break;
    }
}

void rxProgrammeChange(uint8_t channel, uint8_t programme)
{
    switch (static_cast<MidiComms::ProgrammeChangeTypes>(programme))
    {
        case MidiComms::ProgrammeChangeTypes::SAVE_TO_EEPROM:
            writeEepromParameters();
            break;
        case MidiComms::ProgrammeChangeTypes::RESET_TO_DEFAULT_PARAMETERS:
            resetToDefaultParameters();
            break;
        case MidiComms::ProgrammeChangeTypes::CALIBRATE_DAMPED:
            force_sensing.calibrateDamped();
            break;
        case MidiComms::ProgrammeChangeTypes::CALIBRATE_UNDAMPED:
            force_sensing.calibrateUndamped();
            break;
        default:
            printf("Unknown MIDI programme change (%d) received\r\n", programme);
            break;
    }
}

void rxControlChange(uint8_t channel, uint8_t control_number, uint8_t control_value)
{
    switch (static_cast<MidiComms::ControlChangeTypes>(control_number))
    {
        case MidiComms::ControlChangeTypes::TONE_LEVEL:
            if (current_error_state == ErrorStates::DEBUG)
            {
                printf("MIDI CC - Tone Level: %fdB (raw CC val: %d)\r\n", dBToLin(  ((sample_t) control_value - 127.0)), control_value);
            }
            transducer_processing.setResonanceToneLevelDb( (sample_t) control_value - 127.0);
            break;
        default:
            printf("Unknown MIDI control change (%d) received\r\n", control_number);
            break;
    }
}

void txForceSenseVal(sample_t force_sense_val)
{
    uint8_t force_sense_byte = static_cast<uint8_t>(127 * force_sense_val);
    usbMIDI.sendControlChange(static_cast<uint8_t>(MidiComms::ControlChangeTypes::TX_FORCE_SENSE), force_sense_byte, 1);
}

void blinkLED() {
    static bool led_state = false;
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);
}

void setResonantFrequency(sample_t resonant_frequency_hz)
{
    current_cancellation_setup.resonant_frequency_hz = resonant_frequency_hz;
    force_sensing.setResonantFrequencyHz(resonant_frequency_hz);
    transducer_processing.setResonantFrequencyHz(resonant_frequency_hz);
    transducer_processing.setOscillatorFrequencyHz(resonant_frequency_hz);
}

void setToneLevel(sample_t tone_level_db)
{
    current_cancellation_setup.resonance_tone_level_db = tone_level_db;
    transducer_processing.setResonanceToneLevelDb(tone_level_db);
}

void setResonanceQ(sample_t resonance_q)
{
    current_cancellation_setup.resonance_q = resonance_q;
    transducer_processing.setResonanceQ(resonance_q);
}

void setWidebandGain(sample_t wideband_gain_db)
{
    current_cancellation_setup.transducer_input_wideband_gain_db = wideband_gain_db;
    transducer_processing.setTransducerInputWidebandGainDb(wideband_gain_db);
}

void setResonanceGain(sample_t resonance_gain_db)
{
    current_cancellation_setup.resonance_peak_gain_db = resonance_gain_db;
    transducer_processing.setResonancePeakGainDb(resonance_gain_db);
}

void setOutputLowpassFc(sample_t cutoff_frequency_hz)
{
    current_cancellation_setup.output_to_transducer_lpf_cutoff_hz = cutoff_frequency_hz;
    transducer_processing.setOutputLpfFrequencyHz(cutoff_frequency_hz);
}

void setInputLowpassFc(sample_t cutoff_frequency_hz)
{
    current_cancellation_setup.input_from_transducer_lpf_cutoff_hz = cutoff_frequency_hz;
    transducer_processing.setInputLpfFrequencyHz(cutoff_frequency_hz);
}

void resetToDefaultParameters()
{
    current_cancellation_setup.resonant_frequency_hz = RESONANT_FREQ_HZ;
    current_cancellation_setup.resonance_peak_gain_db = -18.3;
    current_cancellation_setup.resonance_q = 10.0;
    current_cancellation_setup.resonance_tone_level_db = -50.0;
    current_cancellation_setup.inductance_filter_coefficient = 0.5;
    current_cancellation_setup.transducer_input_wideband_gain_db = 0.0;
    current_cancellation_setup.sample_rate_hz = AUDIO_SAMPLE_RATE_EXACT;
    current_cancellation_setup.amplifier_type = TransducerFeedbackCancellation::AmplifierType::CURRENT_DRIVE;
    current_cancellation_setup.lowpass_transducer_io = true;
    current_cancellation_setup.output_to_transducer_lpf_cutoff_hz = 10000.0;
    current_cancellation_setup.input_from_transducer_lpf_cutoff_hz = 10000.0;
    transducer_processing.setOscillatorFrequencyHz(RESONANT_FREQ_HZ);
    transducer_processing.setup(current_cancellation_setup);

    force_sensing.setResonantFrequencyHz(RESONANT_FREQ_HZ);
    force_sensing.setWindowSizePeriods(10);

    printf("Reset parameters to defaults. Resonant frequency now %f\r\n",current_cancellation_setup.resonant_frequency_hz);
}

void sendSerialDetails()
{
    printCurrentTime();
    printf(": Device details:\r\n");
    printf("Serial number: %d\r\n",serial_number);
    printf("Project compiled on %s at %s\r\n",__DATE__, __TIME__);
    printf("Project version %d.%d\r\n", VERSION_MAJ, VERSION_MIN);
    printf("Version notes: %s\r\n",VERSION_NOTES);
    printf("Current resonant frequency: %fHz\r\n",current_cancellation_setup.resonant_frequency_hz);
}
