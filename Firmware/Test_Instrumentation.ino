#define PWM1 1
#define PWM2 2
#define PWM3 3
#define PWM4 4
const int pwmPins[] = {PWM1, PWM2, PWM3, PWM4};
const int pwmChannels[] = {0, 1, 2, 3};
const int pwmFrequency = 5000;

void setup() {
    //ties channels to pins
    for (int i = 0; i < 4; i++) {
        ledcSetup(pwmChannels[i], pwmFrequency, 8);
        ledcAttachPin(pwmPins[i], pwmChannels[i]);
        ledcWrite(pwmChannels[i], 128); // Set duty cycle to 50%
    }
}

void loop() {
  // put your main code here, to run repeatedly:

}
