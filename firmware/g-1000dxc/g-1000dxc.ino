// An ESP32-based controller for the Yeasu G-1000DXC antenna rotator
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

// The following Python program was used to fit a Polynomial to the measured
// values using the least squares method. The coefficients and the domain of the
// generated polynomial must be passed as arguments to the PolynomialInput
// constructors used in this source file.
//
//     import numpy as np
//     import matplotlib.pyplot as plt
//     import scipy.stats as stats
//     from numpy.polynomial import Polynomial
//
//     x = [...]
//     y = [...]
//
//     curve = Polynomial.fit(x, y, 8)
//     print(repr(curve))
//     print(curve)
//
//     yf = [curve(x) for x in x]
//
//     plt.plot(x, y)
//     plt.plot(x, yf)
//     plt.show()

// TODO
// * Automatically scale PWM output values depending on the PWM resolution
// * Design a better web UI and API for calibration
// * Make critical parameters run-time configurable and stored in NVM
// * Implement OTA firmware update
// * Add mapping classes between 0-450 and 0-360


// ======== The following values may need to be configured ========

//#define WIFI_SSID     "Queeg"
//#define WIFI_PASSWORD "spravedlnosti14"
//#define MDNS_NAME     "rotator"

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

#define MAX_ORIENTATION (360 + OVERLAP)

#define NO_ORIENTATION -1

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
    int16_t orientation;
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
private:
    uint16_t current;

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
        current = sum / oversample;
        return current;
    }

    virtual int get_raw() override {
        return current;
    }
};


class PolynomialFit: public AnalogInput {
private:
    AnalogInput &input;
    float offset;
    float scale;
    const vector<float> coefficients;

public:
    PolynomialFit(AnalogInput &input, const array<int, 2> &domain, const vector<float> &coefficients):
        input(input), coefficients(coefficients) {
        const auto window = array<int, 2>{-1, 1};

        // TODO: Initialize AnalogInput with minimum / maximum computed from the domain.

        // Compute the parameters for a linear translation of input values from
        // <domain> to a window of <-1, 1>. The coefficients passed in the
        // parameter coefficient must have been fitted for the window, not for
        // the domain.
        float dlen = domain[1] - domain[0];
        float wlen = window[1] - window[0];
        offset = ((float)domain[1] * (float)window[0] - (float)domain[0] * (float)window[1]) / dlen;
        scale = wlen / dlen;
    }

    virtual int get() override {
        // Linearly translate the input value to the window.
        float x = input.get() * scale + offset;

        // Compute the value using Horner's method.
        double sum = coefficients.back();
        for (int i = coefficients.size() - 2; i >= 0; i--)
            sum = coefficients[i] + sum * x;

        return (int)round(sum);
    }

    virtual int get_raw() override {
        return input.get_raw();
    }
};


class Clamped: public AnalogInput {
private:
    AnalogInput &input;

public:
    Clamped(AnalogInput &input, int min, int max): AnalogInput(min, max), input(input) {}

    virtual int get() override {
        int x = input.get();
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    virtual int get_raw() override {
        return input.get_raw();
    }
};


class Rotator {
private:
    VariableSpeedMotor &motor;
    AnalogInput &sensor;

public:
    const int min;
    const int max;

    Rotator(VariableSpeedMotor &motor, AnalogInput &sensor, int min = INT_MIN, int max = INT_MAX):
        motor(motor), sensor(sensor), min(min), max(max) {
    }

    void stop(void) {
        motor.stop();
    }

    void cw() {
        if (sensor.get() < max) motor.cw();
        else stop();
    }

    void ccw() {
        if (sensor.get() > min) motor.ccw();
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


#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
static void init_wifi()
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
#endif


#if defined(MDNS_NAME)
static void init_mdns()
{
    Serial.print("Starting mDNS...");
    if (!MDNS.begin(MDNS_NAME)) {
        Serial.println("failed.");
        return;
    }
    Serial.println("done.");
    Serial.print("mDNS name: ");
    Serial.println(MDNS_NAME);

    MDNS.addService("http", "tcp", 80);
}
#endif


static uint16_t orientation_to_azimuth(uint16_t v)
{
    if (v < 180) return v + 180;
    else if (v >= 180 && v < 360) return v - 180;
    else return v - 180;
}


static uint16_t azimuth_to_orientation(uint16_t v)
{
    if (v < 180) return v + 180;
    else if (v >= 180 && v < 360) return v - 180;
    else return v;
}


static String processor(const String& var)
{
    if (var == "ANTENNA") return String(orientation_to_azimuth(inputs.rotator));
    if (var == "ANTENNA_RAW") return String(rotator->get_raw());

    if (var == "INDICATOR") return String(orientation_to_azimuth(inputs.indicator));
    if (var == "INDICATOR_RAW") return String(indicator->get_raw());

    if (var == "PRESET_KNOB") return String(orientation_to_azimuth(inputs.preset));
    if (var == "PRESET_KNOB_RAW") return String(preset->get_raw());

    if (var == "SPEED") return String(inputs.speed);
    if (var == "SPEED_RAW") return String(speed->get_raw());

    if (var == "PRESET") {
        if (inputs.orientation == NO_ORIENTATION) {
            return String();
        } else {
            return String(orientation_to_azimuth(inputs.orientation));
        }
    }
    return String();
}


static void init_http()
{
    static AsyncWebServer web_server(80);

    web_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    web_server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/style.css", "text/css");
    });

    web_server.on("/set-azimuth", HTTP_GET, [](AsyncWebServerRequest * request) {
        if (request->hasParam("azimuth")) {
            String v = request->getParam("azimuth")->value();
            v.trim();
            inputs.orientation = v.length() == 0 ? NO_ORIENTATION : azimuth_to_orientation(stoi(v.c_str()));
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
    inputs.orientation = NO_ORIENTATION;

#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
    init_wifi();
#if defined(MDNS_NAME)
    init_mdns();
#endif
    init_http();
#endif

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

    // The following values were measured on the physical preset button. The
    // first row contains raw sensor values. The second row contains preset
    // orientations from 0 to 450.
    //
    // 299 | 323 | 347 | 373 | 399 | 425 | 448 | 472 | 495 | 518 | 542 | 564 | 584 | 610 | 631 | 651
    //   0 |  30 |  60 |  90 | 120 | 150 | 180 | 210 | 240 | 270 | 300 | 330 | 360 | 390 | 420 | 450
    //
    // Fitting a polynomial to the measured values produced the following
    // parameters:
    static PolynomialFit preset_fitted(preset_adc, array<int, 2>{ 299, 651 }, vector<float>{
        214.13017251512497, 225.77818898710638, -0.8075325747516042,
        -9.47189296911141, 98.44743275442832, -1.040438358931594,
        -185.95051207672086, 9.813429808629284, 99.26608067691565
    });
    static Clamped preset_clamped(preset_fitted, 0, MAX_ORIENTATION);
    preset = &preset_clamped;

    // The speed knob is a potentiometer linearly mapped to the <0, 255> range.
    static Adc speed_adc(SPEED_SENSOR, 256);
    static PolynomialFit speed_fitted(speed_adc, array<int, 2>{ 307, 638 }, vector<float>{
        127.49999999999997, 127.50000000000001
    });
    static Clamped speed_clamped(speed_fitted, 0, 255);
    speed = &speed_clamped;

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

    // 322 | 329 | 335 | 343 | 350 | 356 | 364 | 370 | 378 | 386
    //   0 |  10 |  20 |  30 |  40 |  50 |  60 |  70 |  80 |  90

    // 392 | 400 | 407 | 413 | 420 | 426 | 434 | 440 | 446 | 453
    // 100 | 110 | 120 | 130 | 140 | 150 | 160 | 170 | 180 | 190

    // 460 | 467 | 473 | 480 | 487 | 493 | 500 | 507 | 515 | 522
    // 200 | 210 | 220 | 230 | 240 | 250 | 260 | 270 | 280 | 290

    // 528 | 534 | 541 | 549 | 555 | 562 | 569 | 575 | 582 | 589
    // 300 | 310 | 320 | 330 | 340 | 350 | 360 | 370 | 380 | 390

    // 595 | 602 | 610 | 618 | 624 | 630
    // 400 | 410 | 420 | 430 | 440 | 450
    static PolynomialFit indicator_sensor_fitted(indicator_sensor_adc, array<int, 2>{ 322, 630}, vector<float>{
        224.32983151113456, 227.975502016654, -23.318904870464237,
        2.928610614473806, 105.82445830796173, -21.214288612988362,
        -149.52566528385233, 15.029923558472099, 67.52449512618676
    });
    static Rotator indicator_(indicator_motor, indicator_sensor_fitted, 0, MAX_ORIENTATION);
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

    // The following values were measured on the physical antenna rotator. The
    // first row contains raw rotator sensor values. The second row contains
    // antenna's orientations from 0 to 450.
    //
    // 20 | 76 | 126 | 176 | 234 | 294 | 356 | 417 | 480 | 532 | 577 | 637 | 698 | 759 | 828 | 900
    //  0 | 30 |  60 |  90 | 120 | 150 | 180 | 210 | 240 | 270 | 300 | 330 | 360 | 390 | 420 | 450
    //
    // A polynomial was fitted using the least squares method to the measured
    // value. The fit produced the following parameters:
    static PolynomialFit rotator_sensor_fitted(rotator_sensor_adc, array<int, 2>{ 20, 900 }, vector<float>{
        231.06693256776882, 236.21224191173644, 73.9423590843075,
        -50.288648819706715, -238.6488923001425, 89.08520518133807,
        251.9955496837724, -49.982680307885396, -93.3990214432426
    });
    static Rotator rotator_(rotator_motor, rotator_sensor_fitted);
    rotator = &rotator_;
}


static void handle_overlap()
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


static void handle_speed()
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
    // import numpy as np
    // import matplotlib.pyplot as plt
    // import scipy.stats as stats
    //
    // x = [0, 63, 127, 191, 255]
    // y = [32, 36, 47, 66, 75]
    //
    // curve = np.poly1d(np.polyfit(x, y, 4))
    // print(curve)
    //
    // y2 = [curve(x) for x in x]
    //
    // plt.plot(x, y)
    // plt.plot(x, y2)
    // plt.show()

    int x = speed->get();
    rotator->set_speed(x);
    double v = -4.741e-08 * x * x * x * x + 1.871e-05 * x * x * x -
        0.001369 * x * x + 0.08733 * x + 32;
    indicator->set_speed(v);
}


static void handle_preset_button()
{
    // If the preset button was pressed, transfer the preset orientation value
    // from the input to the internal orientation state variable which will make
    // the rotator start orienting the antenna toward that orientation. The
    // operation can be canceled by pressing either the cw or ccw button.

    bool preset_pressed = !previous_inputs.preset_button && inputs.preset_button;
    if (preset_pressed) inputs.orientation = inputs.preset;
}


static void rotate_to_orientation() {
    if (inputs.orientation != NO_ORIENTATION) {
        if (inputs.rotator < inputs.orientation) {
            indicator->cw();
            rotator->cw();
        } else if (inputs.rotator > inputs.orientation) {
            indicator->ccw();
            rotator->ccw();
        } else {
            inputs.orientation = NO_ORIENTATION;
            indicator->stop();
            rotator->stop();
        }
    }

    // This can be triggered by the user trying to stop the rotator by
    // submitting a "NO_ORIENTATION" value through the web interface.
    if (previous_inputs.orientation != NO_ORIENTATION && inputs.orientation == NO_ORIENTATION) {
        indicator->stop();
        rotator->stop();
    }

    // If we are chasing a preset orientation, check whether we have reached the
    // target and in that case turn the rotator and indicator off.
    if (inputs.orientation != NO_ORIENTATION) {
        if (inputs.ccw_button || inputs.cw_button) {
            inputs.orientation = NO_ORIENTATION;
            indicator->stop();
            rotator->stop();
        } else {
            switch(indicator->state()) {
                case MOVING_CW:
                    if (inputs.indicator >= inputs.orientation) {
                        inputs.orientation = NO_ORIENTATION;
                        indicator->stop();
                        rotator->stop();
                    }
                    break;

                case MOVING_CCW:
                    if (inputs.indicator <= inputs.orientation) {
                        inputs.orientation = NO_ORIENTATION;
                        indicator->stop();
                        rotator->stop();
                    }
                    break;
            }
        }
    }
}


static void handle_buttons()
{
    // Only respond to CW and CCW buttons if we are not rotating to a preset
    // orientation.
    if (inputs.orientation != NO_ORIENTATION) return;

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


static void handle_sync()
{
    // If none of the buttons is pressed and we do not have a preset orientation,
    // check that the position of the rotator matches the position of the
    // indicator and if not, adjust the indicator.
    if (!inputs.cw_button && !inputs.ccw_button && inputs.orientation == NO_ORIENTATION) {
        if (inputs.indicator < (inputs.rotator - INDICATOR_TOLERANCE)) {
            indicator->cw();
        } else if (inputs.indicator > (inputs.rotator + INDICATOR_TOLERANCE)) {
            indicator->ccw();
        } else {
            indicator->stop();
        }
    }
}


static void read_inputs(void)
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
    handle_preset_button();
    rotate_to_orientation();
    handle_buttons();
    handle_sync();

    // Save the current state of inputs for the next run
    led->set(false);
    memcpy(&previous_inputs, &inputs, sizeof(inputs));
}
