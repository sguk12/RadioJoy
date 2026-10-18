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
    uint32_t lastToggle = 0;
    static const uint32_t PULSE_MS = 30; // min press and release time per encoder step

public:
    // Constructor that creates a new encoder
    // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
    // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
    // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
    RotaryEncoderWrapper(int pinA, int pinB, RotaryEncoder::LatchMode mode)
        : reportedPosition(0), encoderPosition(0), buttonCCWState(BUTTON_UP), buttonCWState(BUTTON_UP), ownsEncoder(true) {
        encoder = new RotaryEncoder(pinA, pinB, mode); // also sets INPUT_PULLUP on both pins
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
    
    // Update positions and return true if a button state changed.
    // Each encoder step becomes one press held for PULSE_MS followed by a release held for PULSE_MS,
    // so the host sees it even when it polls slower than the USB report rate. Fast spins are queued.
    bool update() {
        if (!encoder) return false;
        if (millis() - lastToggle < PULSE_MS) return false; // hold the current press/release

        if (buttonCWState == BUTTON_DOWN || buttonCCWState == BUTTON_DOWN) {
            buttonCWState = BUTTON_UP;
            buttonCCWState = BUTTON_UP;
            lastToggle = millis();
            return true;
        }

        encoderPosition = getPosition();
        if (encoderPosition > reportedPosition) {
            buttonCWState = BUTTON_DOWN;
            reportedPosition++;
            lastToggle = millis();
            return true;
        }
        if (encoderPosition < reportedPosition) {
            buttonCCWState = BUTTON_DOWN;
            reportedPosition--;
            lastToggle = millis();
            return true;
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
        encoder->setPosition(0);
        encoderPosition = getPosition();
        reportedPosition = encoderPosition;
        buttonCWState = BUTTON_UP;
        buttonCCWState = BUTTON_UP;
        lastToggle = 0;
    }

    // Get raw encoder pointer (use carefully)
    RotaryEncoder* getEncoder() {
        return encoder;
    }
};

#endif // ROTARY_ENCODER_WRAPPER_H