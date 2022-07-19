// An ESP32-based controller for the Yeasu G-1000DX antenna rotator
//
// Copyright 2022 Jan Janak <jan@janakj.org>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


// TODO
// * Automatically scale PWM output values depending on the PWM resolution
// * Design a better web UI and API for calibration
// * Make critical parameters run-time configurable and stored in NVM
// * Implement OTA firmware update
// * Add mapping classes between 0-450 and 0-360


// ======== The following values may need to be configured ========

#define WIFI_SSID "<wifi ssid>"
#define WIFI_PASSWORD "<wifi password>"
#define MDNS_NAME "<zeroconf name>"

// The size of the overlap region in degrees
#define OVERLAP 90

// Calculate ADC values as rolling average over this number of samples
#define OVERSAMPLE 256

// PWM output frequency in Hz. Note that the L293D chip can handle no more than
// roughly 5000 Hz.
#define PWM_FREQUENCY 100

// The resolution of PWM outputs in bits
#define PWM_RESOLUTION 8

// We allow the indicator to be misaligned with the rotator by this number of
// degrees in either direction.
#define INDICATOR_TOLERANCE 3

// The ADC1 one peripheral that we use to read all analog inputs is configured
// with a 10-bit resolution that gives us 1024 values in total in the range <0,
// 1023>. The value configured here must be between 8 and 12.
#define ADC_RESOLUTION 10

// Wait the given number of milliseconds for the state of buttons to stabilize.
#define BUTTON_DEBOUNCE_INTERVAL 20

// The mapping of various controller components to GPIO pins on the Adafruit
// ESP32 Feather board.
#define INDICATOR_CW       4
#define LED               13
#define INDICATOR_SPEED   14  // LED PWM channel 0
#define PRESET_SWITCH     16
#define CCW_BUTTON        18
#define CW_BUTTON         19
#define OVERLAP_INDICATOR 21
#define ROTATOR_CCW       22
#define ROTATOR_CW        23
#define SPEED_CONTROL     25  // DAC 1
#define INDICATOR_CCW     26
#define ROTATOR_SENSOR    33  // ADC1 channel 5
#define SPEED_SENSOR      34  // ADC1 channel 6
#define PRESET_SENSOR     36  // ADC1 channel 0
#define INDICATOR_SENSOR  39  // ADC1 channel 3

// ======== The code below typically shouldn't require changes ========


#include <SPIFFS.h>
#include <string>
#include <deque>
#include <driver/dac.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

#define MAX_AZIMUTH (360 + OVERLAP)

#define NO_AZIMUTH -1

using namespace std;


typedef enum state {
    IDLE,
    MOVING_CW,
    MOVING_CCW
} state_t;


typedef struct inputs {
    int preset_button : 1;
    int cw_button : 1;
    int ccw_button : 1;
    int16_t preset;
    int16_t speed;
    int16_t rotator;
    int16_t indicator;
    int16_t azimuth;
} inputs_t;


int64_t get_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
}


class DigitalInput {
public:
    virtual bool get() = 0;
};


class DigitalOutput {
public:
    virtual void set(bool state) = 0;
};


class DigitalIO: public DigitalInput, public DigitalOutput {
};


class AnalogInput {
public:
    const int min;
    const int max;
    AnalogInput(int min=INT_MIN, int max=INT_MAX): min(min), max(max) {};
    virtual int get() = 0;
    virtual int get_raw() = 0;
};


class AnalogOutput {
public:
    virtual void set(int value) const = 0;
};


class AnalogIO: public AnalogInput, public AnalogOutput {
};


class DebouncedInput: public DigitalInput {
protected:
    DigitalInput &input;
    unsigned int delay;
    bool state;
    bool previous_state;
    int64_t last_change;

public:
    DebouncedInput(DigitalInput &input, unsigned delay=BUTTON_DEBOUNCE_INTERVAL): input(input) {
        this->delay = delay;
        previous_state = input.get();
        last_change = -1;
    }

    virtual bool get() override {
        int64_t now = get_time();
        bool state = input.get();

        if (state != previous_state)
            last_change = now;

        if (last_change == -1 || (now - last_change) >= delay)
            this->state = state;

        previous_state = state;
        return this->state;
    }
};


class Gpio: public DigitalIO {
private:
    uint8_t gpio;
    uint8_t mode;

public:
    Gpio(uint8_t gpio, uint8_t mode=GPIO_MODE_INPUT_OUTPUT) {
        this->gpio = gpio;
        this->mode = mode;
        pinMode(gpio, mode);
    }

    virtual void set(bool state) override {
        digitalWrite(gpio, state ? HIGH : LOW);
    }

    virtual bool get() override {
        int v = digitalRead(gpio);
        switch (mode) {
            case INPUT_PULLUP:
                return v == 0;

            default:
                return v == 1;
        }
    }
};


class Motor {
private:
    DigitalIO &cw_gpio;
    DigitalIO &ccw_gpio;
    const int wait;

public:
    Motor(DigitalIO &cw, DigitalIO &ccw, int wait=0): cw_gpio(cw), ccw_gpio(ccw), wait(wait) {
        stop();
    }

    void stop() {
        if (cw_gpio.get() || ccw_gpio.get()) {
            cw_gpio.set(false);
            ccw_gpio.set(false);
            delay(wait);
        }
    }

    void cw() {
        if (ccw_gpio.get()) {
            ccw_gpio.set(false);
            delay(wait);
        }
        cw_gpio.set(true);
    }

    state_t state() {
        if (cw_gpio.get()) return MOVING_CW;
        if (ccw_gpio.get()) return MOVING_CCW;
        return IDLE;
    }

    void ccw() {
        if (cw_gpio.get()) {
            cw_gpio.set(false);
            delay(wait);
        }
        ccw_gpio.set(true);
    }
};


class VariableSpeedMotor: public Motor {
private:
    const AnalogOutput &speed_control;

public:
    VariableSpeedMotor(DigitalIO &cw, DigitalIO &ccw, AnalogOutput &speed):
        Motor(cw, ccw), speed_control(speed) {
    }

    void set_speed(int value) {
        speed_control.set(value);
    }
};


class Dac: public AnalogOutput {
private:
    uint8_t gpio;
    dac_channel_t channel;

public:
    Dac(uint8_t gpio, dac_channel_t channel) {
        this->gpio = gpio;
        this->channel = channel;
        pinMode(gpio, ANALOG);
        dac_output_enable(channel);
    }

    virtual void set(int value) const override {
        dac_output_voltage(channel, value);
    }
};


class Pwm: public AnalogOutput {
private:
    uint8_t gpio;
    uint8_t channel;

public:
    Pwm(uint8_t gpio, uint8_t channel, int frequency=PWM_FREQUENCY, int resolution=PWM_RESOLUTION) {
        this->gpio = gpio;
        this->channel = channel;
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(gpio, channel);
    }

    virtual void set(int value) const override {
        ledcWrite(this->channel, value);
    }
};


class Adc: public AnalogInput {
protected:
    uint8_t gpio;
    int oversample;
    deque<uint16_t> history;
    uint64_t sum;

public:
    Adc(uint8_t gpio): Adc(gpio, OVERSAMPLE) {}

    Adc(uint8_t gpio, int oversample): AnalogInput(0, (1 << ADC_RESOLUTION) - 1) {
        this->gpio = gpio;
        this->oversample = oversample;
        pinMode(gpio, ANALOG);

        sum = 0;
        for (int i = 0; i < oversample; i++) {
            history.push_back(analogRead(gpio));
            sum += history.back();
        }
    }

    virtual int get() override {
        sum -= history.front();
        history.pop_front();

        history.push_back(analogRead(gpio));
        sum += history.back();
        return sum / oversample;
    }

    virtual int get_raw() override {
        return analogRead(gpio);
    }
};


class LinearInput: public AnalogInput {
private:
    AnalogInput &input;
    const array<int,2> p1;
    const array<int,2> p2;
    int x;

public:
    LinearInput(AnalogInput &input, const array<int,2> &p1, const array<int, 2> &p2, int min=INT_MIN, int max=INT_MAX):
        AnalogInput(min, max), input(input), p1(p1), p2(p2) { }

    virtual int get() override {
        x = input.get();
        int y = (p1[1] * (p2[0] - x) + p2[1] * (x - p1[0])) / (p2[0] - p1[0]);
        if (y < min) y = min;
        if (y > max) y = max;
        return y;
    }

    virtual int get_raw() override {
        return x;
    }
};


class Rotator {
private:
    VariableSpeedMotor &motor;
    AnalogInput &sensor;

public:
    const int min;
    const int max;

    Rotator(VariableSpeedMotor &motor, AnalogInput &sensor):
        motor(motor), sensor(sensor), min(0), max(MAX_AZIMUTH) {
    }

    void stop(void) {
        motor.stop();
    }

    void cw() {
        if (sensor.get() < sensor.max) motor.cw();
        else stop();
    }

    void ccw() {
        if (sensor.get() > sensor.min) motor.ccw();
        else stop();
    }

    state_t state() {
        return motor.state();
    }

    int get(void) const {
        return sensor.get();
    }

    void set_speed(int speed) {
        motor.set_speed(speed);
    }

    int get_raw() const {
        return sensor.get_raw();
    }
};


static DigitalOutput *led;
static DigitalOutput *overlap_indicator;
static DigitalInput *preset_button;
static DigitalInput *cw_button;
static DigitalInput *ccw_button;
static Rotator *rotator, *indicator;
static AnalogInput *preset;
static AnalogInput *speed;


static inputs_t inputs, previous_inputs;


void init_wifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi '");
    Serial.print(WIFI_SSID);
    Serial.print("'...");
    Serial.flush();
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(1000);
    }
    Serial.println("done.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


void init_mdns()
{
    Serial.print("Starting mDNS...");
    if (!MDNS.begin(MDNS_NAME)) {
        Serial.println("failed.");
        return;
    }
    Serial.println("done.");
    Serial.print("mDNS name: ");
    Serial.println(MDNS_NAME);
}


String processor(const String& var)
{
    if (var == "ANTENNA") return String(inputs.rotator);
    if (var == "ANTENNA_RAW") return String(rotator->get_raw());
    if (var == "INDICATOR") return String(inputs.indicator);
    if (var == "INDICATOR_RAW") return String(indicator->get_raw());
    if (var == "PRESET") return String(inputs.preset);
    if (var == "SPEED") return String(inputs.speed);
    if (var == "AZIMUTH") return String(inputs.azimuth);
    return String();
}


void init_http()
{
    MDNS.addService("http", "tcp", 80);

    static AsyncWebServer web_server(80);

    web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    web_server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    web_server.on("/set", HTTP_GET, [](AsyncWebServerRequest * request) {
        if (request->hasParam("azimuth")) {
            String v = request->getParam("azimuth")->value();
            inputs.azimuth = stoi(v.c_str());
        }
        request->redirect("/");
    });

    web_server.begin();
}


void setup()
{
    Serial.begin(115200);
    Serial.println("Starting Yeasu G-1000DXC controller");

    // The ADC inputs on the ESP32 are fairly susceptible to noise. To mitigate
    // some of the noise, we reconfigure the ADC1 peripheral to 10 bits (values
    // from 0 to 1023). We have a voltage divider on each potentiometer that
    // makes roughly one third of the ADC range available. That range is then
    // mapped into the 0 to 450 degrees range which provides slightly over one
    // degree per ADC value. That should be sufficient and is more than what the
    // indicator can display.
    //
    // Note: Adding capacitors to ADC inputs and lowering the resolution isn't
    // sufficient to mititage noise. One also needs to oversample (average) ADC
    // values over a large number of samples, at least 64 and prerably 256,
    // depending on how often the main loop runs.

    analogReadResolution(ADC_RESOLUTION);
    analogSetWidth(ADC_RESOLUTION);

    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    memset(&inputs, '\0', sizeof(inputs));
    inputs.azimuth = NO_AZIMUTH;

    // init_wifi();
    // init_mdns();
    // init_http();

    static Gpio preset_gpio(PRESET_SWITCH, INPUT_PULLUP);
    static Gpio ccw_gpio(CCW_BUTTON, INPUT_PULLUP);
    static Gpio cw_gpio(CW_BUTTON, INPUT_PULLUP);
    static DebouncedInput
        preset_button_(preset_gpio),
        cw_button_(cw_gpio),
        ccw_button_(ccw_gpio);

    preset_button = &preset_button_;
    cw_button = &cw_button_;
    ccw_button = &ccw_button_;

    // A red LED on the Feather board. We use the LED as an activity indicator.
    // It is turned on while the main loop (below) is running to match inputs to
    // outputs.
    static Gpio led_(LED);
    led = &led_;

    // The Yeasu G-1000DXC rotator has a 90 degree overlap, i.e., it can move
    // between 0 and 450 degress. The overlap indicator is a LED connected to
    // the Feather board that is turned on while the rotator is within the
    // overlap region.
    static Gpio overlap_indicator_(OVERLAP_INDICATOR);
    overlap_indicator = &overlap_indicator_;

    // The preset knob is a potentiometer linearly mapped to the range <0, 450>.
    // The linear mapping isn't ideal, either the ADC or the potentiometer is
    // slightly non-linear, but it will do.
    static Adc preset_adc(PRESET_SENSOR);
    static LinearInput preset_(preset_adc, array<int, 2>{302, 0},
        array<int, 2>{655, MAX_AZIMUTH}, 0, MAX_AZIMUTH);
    preset = &preset_;

    // The speed knob is a potentiometer linearly mapped to the <0, 255> range.
    static Adc speed_adc(SPEED_SENSOR);
    static LinearInput speed_(speed_adc, array<int, 2>{307, 0}, array<int, 2>{638, 255}, 0, 255);
    speed = &speed_;

    // The indicator consists of two GPIO output pins connected to the L293D
    // driver chip that rotate the indicator motor clockwise or
    // counter-clockwise, GPIO pin in PWM mode that control the indicator
    // motor's speed via one of the "enable" inputs on the L293D, and an ADC
    // input connected to the indicator's potentiometer. Beware that the
    // potentiometer has a limited physical range and can only move slightly
    // beyond the interval <-200, +290>, but not by much. The indicator sensor
    // is linearly mapped to the range <0, 450> and we set a hard limit on that
    // range so that it could not be moved beyond the potentiometer's physical
    // range.
    static Gpio indicator_cw_gpio(INDICATOR_CW);
    static Gpio indicator_ccw_gpio(INDICATOR_CCW);
    static Pwm indicator_speed(INDICATOR_SPEED, 0);
    static VariableSpeedMotor indicator_motor(indicator_cw_gpio, indicator_ccw_gpio, indicator_speed);

    static Adc indicator_sensor_adc(INDICATOR_SENSOR);
    static LinearInput indicator_sensor(indicator_sensor_adc, array<int, 2>{322, 0},
        array<int, 2>{630, MAX_AZIMUTH}, 0, MAX_AZIMUTH);
    static Rotator indicator_(indicator_motor, indicator_sensor);
    indicator = &indicator_;

    // The rotator consists of two GPIO output pins connected to the L293D
    // driver chip (which drives rotator motor relays), a DAC output connected
    // to an opamp that regulates the volage for the rotator motor, and ADC
    // input connected to the rotator potentimeter. The potentiometer is
    // linearly mapped to roughly the 0-450 range (the rotator's range is
    // narrower by a few degrees).
    static Gpio motor_cw_gpio(ROTATOR_CW);
    static Gpio motor_ccw_gpio(ROTATOR_CCW);
    static Dac speed_control(SPEED_CONTROL, DAC_CHANNEL_1);
    static VariableSpeedMotor rotator_motor(motor_cw_gpio, motor_ccw_gpio, speed_control);
    static Adc rotator_sensor_adc(ROTATOR_SENSOR);
    static LinearInput rotator_sensor(rotator_sensor_adc, array<int, 2>{14, 5},
        array<int, 2>{901, MAX_AZIMUTH - 5});
    static Rotator rotator_(rotator_motor, rotator_sensor);
    rotator = &rotator_;
}


void handle_overlap()
{
    // Turn the overlap indicator on if we get within half of the overlap band
    // from either side. The original controller would turn the indicator on
    // only if the rotator moves past 360, i.e., it indicated that you can no
    // longer move clock-wise, but it did not indicate that one could not move
    // counter clock-wise.
    bool onoff = abs(inputs.indicator - indicator->min) < OVERLAP / 2 ||
        abs(inputs.indicator - indicator->max) < OVERLAP / 2;
    overlap_indicator->set(onoff);
}


void handle_speed()
{
    // This function attempts to match the speed of the rotator and indicator
    // motors over the entire range of the speed knob. The following polynomial
    // was generated out of the following table:
    //
    // rotator speed | indicator speed
    // --------------+----------------
    //     255       |     75
    //     191       |     66
    //     127       |     47
    //      63       |     36
    //       0       |     32
    //
    // with the following program:
    //
    // import numpy as np import matplotlib.pyplot as plt import scipy.stats as
    // stats
    //
    // x = [0, 63, 127, 191, 255] y = [32, 36, 47, 66, 75]
    //
    // curve = np.poly1d(np.polyfit(x, y, 4)) print(curve)
    //
    // y2 = [curve(x) for x in x]
    //
    // plt.plot(x, y) plt.plot(x, y2) plt.show()

    int x = speed->get();
    rotator->set_speed(x);
    double v = -4.741e-08 * x * x * x * x + 1.871e-05 * x * x * x -
        0.001369 * x * x + 0.08733 * x + 32;
    indicator->set_speed(v);
}


void handle_preset()
{
    // If the preset button was pressed, transfer the preset azimut value from
    // the input to the internal state which will make the rotator start
    // orienting the antenna toward that azimuth. The operation can be canceled
    // by pressing either the cw or ccw button.

    bool preset_pressed = !previous_inputs.preset_button && inputs.preset_button;

    if (preset_pressed) {
        if (inputs.rotator < inputs.preset) {
            inputs.azimuth = inputs.preset;
            indicator->cw();
            rotator->cw();
        } else if (inputs.rotator > inputs.preset) {
            inputs.azimuth = inputs.preset;
            indicator->ccw();
            rotator->ccw();
        } else {
            inputs.azimuth = NO_AZIMUTH;
            indicator->stop();
            rotator->stop();
        }
    }

    // If we are chasing a preset azimuth, check whether we have reached the
    // target and in that case turn the rotator and indicators off.
    if (inputs.azimuth != NO_AZIMUTH) {
        if (inputs.ccw_button || inputs.cw_button) {
            inputs.azimuth = NO_AZIMUTH;
            indicator->stop();
            rotator->stop();
        } else {
            switch(indicator->state()) {
                case MOVING_CW:
                    if (inputs.indicator >= inputs.azimuth) {
                        inputs.azimuth = NO_AZIMUTH;
                        indicator->stop();
                        rotator->stop();
                    }
                    break;

                case MOVING_CCW:
                    if (inputs.indicator <= inputs.azimuth) {
                        inputs.azimuth = NO_AZIMUTH;
                        indicator->stop();
                        rotator->stop();
                    }
                    break;
            }
        }
    }
}


void handle_buttons()
{
    // Only respond to CW and CCW buttons if we are not rotating to a preset
    // azimuth.
    if (inputs.azimuth != NO_AZIMUTH) return;

    if (inputs.cw_button) {
        rotator->cw();
        indicator->cw();
    }

    if (inputs.ccw_button) {
        rotator->ccw();
        indicator->ccw();
    }

    if (!inputs.cw_button && !inputs.ccw_button) {
        rotator->stop();
        indicator->stop();
    }
}


void handle_sync()
{
    // If none of the buttons is pressed and we do not have a preset azimuth,
    // check that the position of the rotator matches the position of the
    // indicator and if not, adjust the indicator.
    if (!inputs.cw_button && !inputs.ccw_button && inputs.azimuth == NO_AZIMUTH) {
        if (inputs.indicator < (inputs.rotator - INDICATOR_TOLERANCE)) {
            indicator->cw();
        } else if (inputs.indicator > (inputs.rotator + INDICATOR_TOLERANCE)) {
            indicator->ccw();
        } else {
            indicator->stop();
        }
    }
}


void read_inputs(void)
{
    inputs.preset_button = preset_button->get();
    inputs.cw_button = cw_button->get();
    inputs.ccw_button = ccw_button->get();
    inputs.preset = preset->get();
    inputs.speed = speed->get();
    inputs.indicator = indicator->get();
    inputs.rotator = rotator->get();
}


void loop()
{
    // Read inputs. If none of the inputs changed since the previous run, do
    // nothing and return immediately.
    read_inputs();
    if (!memcmp(&inputs, &previous_inputs, sizeof(inputs))) return;
    led->set(true);

    handle_speed();
    handle_overlap();
    handle_preset();
    handle_buttons();
    handle_sync();

    // Save the current state of inputs for the next run
    led->set(false);
    memcpy(&previous_inputs, &inputs, sizeof(inputs));
}
