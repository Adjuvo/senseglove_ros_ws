# Motor location constants (maps to ENova2VibroMotor)
uint8 MOTOR_THUMB_TIP   = 0
uint8 MOTOR_INDEX_TIP   = 1
uint8 MOTOR_PALM_INDEX  = 2
uint8 MOTOR_PALM_PINKY  = 3

# Waveform type constants (maps to EWaveformType)
uint8 WAVE_SINE     = 0
uint8 WAVE_SQUARE   = 1
uint8 WAVE_SAW_UP   = 2
uint8 WAVE_SAW_DOWN = 3
uint8 WAVE_TRIANGLE = 4
uint8 WAVE_NOISE    = 5

# Target LRA motor
uint8   motor_location

# Core waveform
float32 amplitude        # Vibration intensity [0.0 - 1.0]
float32 frequency_start  # Vibration Frequency at the start of the vibration [10 - 500 Hz] (optimal ~180 Hz for Nova 2 LRA)
float32 frequency_end    # Vibration Frequency at the end of the vibration [10 - 500 Hz] (set equal to frequency_start for constant frequency)
uint8   wave_type        # use constants above

# Envelope
float32 attack_time      # Time to reach from 0.0 to Amplitude [0.0 - 1.0 s] (ramp up time)
float32 sustain_time     # Time for which the signal will stay at Amplitude [0.0 - 1.0 s] (hold time at amplitude)
float32 decay_time       # Time to reach from Amplitude down to 0.0 [0.0 - 1.0 s] (ramp down time)

# Frequency sweep (advanced, set frequency_switch_time=0 to disable)
float32 frequency_switch_time    # 0.0 - 1.0, fraction of effect duration at which frequency jumps
float32 frequency_switch_factor  # 1.0 - 3.0, frequency multiplier applied at switch_time

# Repeat
int32   repeat_amount    # 1 - 100
bool    infinite         # continuous playback, overrides repeat_amount