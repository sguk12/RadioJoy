#ifndef ROTARY_ENCODER_WRAPPER_H
#define ROTARY_ENCODER_WRAPPER_H

#include <RotaryEncoder.h>

#define BUTTON_DOWN 1
#define BUTTON_UP 0


class RotaryEncoderWrapper {
private:
    RotaryEncoder* encoder;
    long reportedPosition;
    long encoderPosition;
    uint8_t buttonCCWState;
    uint8_t buttonCWState;
    bool ownsEncoder;

public:
    // Constructor that creates a new encoder
    // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
    // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
    // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
    RotaryEncoderWrapper(int pinA, int pinB, RotaryEncoder::LatchMode mode)
        : reportedPosition(0), encoderPosition(0), buttonCCWState(BUTTON_UP), buttonCWState(BUTTON_UP), ownsEncoder(true) {
        encoder = new RotaryEncoder(pinA, pinB, mode);
    }
    
    // Constructor that wraps an existing encoder
    RotaryEncoderWrapper(RotaryEncoder* existingEncoder)
        : encoder(existingEncoder), reportedPosition(0), encoderPosition(0), buttonCCWState(BUTTON_UP), buttonCWState(BUTTON_UP), ownsEncoder(false) {
    }
    
    // Destructor
    ~RotaryEncoderWrapper() {
        if (ownsEncoder && encoder) {
            delete encoder;
        }
    }
    
    // Tick the encoder (call this in ISR)
    void tick() {
        if (encoder) {
            encoder->tick();
        }
    }
    
    // Get the current encoder position
    long getPosition() {
        return encoder ? encoder->getPosition() : 0;
    }
    
    // Get the direction of rotation
    RotaryEncoder::Direction getDirection() {
        return encoder ? encoder->getDirection() : RotaryEncoder::Direction::NOROTATION;
    }
    
    // Update positions and return true if position changed
    bool update() {
        if (!encoder) return false;
        
        encoderPosition = encoder->getPosition();
        if (encoderPosition > reportedPosition) {
            buttonCCWState = BUTTON_UP;
            buttonCWState = nextButtonState(buttonCWState);
            if (buttonCWState == BUTTON_DOWN){
                reportedPosition++;
            }
            return true;
        }
        if (encoderPosition < reportedPosition) {
            buttonCWState = BUTTON_UP;
            buttonCCWState = nextButtonState(buttonCCWState);
            if (buttonCCWState == BUTTON_DOWN){
                reportedPosition--;
            }
            return true;
        }
        if (encoderPosition == reportedPosition) {
            buttonCWState = BUTTON_UP;
            buttonCCWState = BUTTON_UP;
        }
        return false;
    }

    uint8_t getButtonCCW(){
        return buttonCCWState;
    }
    
    uint8_t getButtonCW(){
        return buttonCWState;
    }
    
    uint8_t nextButtonState(uint8_t state){
        return state? BUTTON_UP: BUTTON_DOWN;
    }
    
    // Get the last reported position
    long getReportedPosition() const {
        return reportedPosition;
    }

    // Reset positions to current encoder position
    void reset() {
        encoderPosition = getPosition();
        reportedPosition = encoderPosition;
        buttonCWState = BUTTON_UP;
        buttonCCWState = BUTTON_UP;
    }

    // Get raw encoder pointer (use carefully)
    RotaryEncoder* getEncoder() {
        return encoder;
    }
};

#endif // ROTARY_ENCODER_WRAPPER_H