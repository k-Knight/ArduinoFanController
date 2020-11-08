#define MAX_SPEED 160
#define COMMAND_MAX_LEN 256
char readData[COMMAND_MAX_LEN];
int failed_commands = 0;
bool connected = false;

int FanMax = 56;
float CurrentSpeed = 0;

void setup() {
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |    // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
        GCLK_GENDIV_ID(4);                    // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);        // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |    // Set the duty cycle to 50/50 HIGH/LOW
        GCLK_GENCTRL_GENEN |                 // Enable GCLK4
        GCLK_GENCTRL_SRC_DFLL48M |           // Set the 48MHz clock source
        GCLK_GENCTRL_ID(4);                  // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);       // Wait for synchronization

    // Enable the port multiplexer for the digital pin D7
    PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;

    // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
    // F & E specify the timers: TCC0, TCC1 and TCC2
    PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |    // Enable GCLK4 to TCC0 and TCC1
        GCLK_CLKCTRL_GEN_GCLK4 |               // Select GCLK4
        GCLK_CLKCTRL_ID_TCC0_TCC1;             // Feed GCLK4 to TCC0 and TCC1*/
    while (GCLK->STATUS.bit.SYNCBUSY);         // Wait for synchronization

    // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
    REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |    // Reverse the output polarity on all TCC0 outputs
        TCC_WAVE_WAVEGEN_DSBOTH;            // Setup dual slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE);        // Wait for synchronization
    
    // Divide the 16MHz signal by 2 giving 8MHz (0.125us) TCC0 timer tick and enable the outputs
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 2
        TCC_CTRLA_ENABLE;                           // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
    
    REG_TCC0_PER = MAX_SPEED;       // 25 kHz
    while(TCC0->SYNCBUSY.bit.PER);

    REG_TCC0_CCB3 = 0;  // 0% duty
    while(TCC0->SYNCBUSY.bit.CCB3);

    Serial.begin(9600);
}

void disconnect() {
    connected = false;
    Serial.write(0x02);
    Serial.write(0x42);
    Serial.write(0x42);
    failed_commands = 0;
}

void connect() {
    Serial.write(0x02);
    Serial.write(0x48);
    Serial.write(0x49);
    connected = true;
}

void reject_msg() {
    Serial.write(0x02);
    Serial.write(0x4e);
    Serial.write(0x4f);
}

long parse_temperature(unsigned char *bytes) {
    long temperature = bytes[0];
    temperature <<= 8;
    temperature += bytes[1];
    temperature <<= 8;
    temperature += bytes[2];
    temperature <<= 8;
    temperature += bytes[3];

    return temperature;
}

void set_fan_speed(long temperature) {
    float duty = 0.0f;

    if (temperature >= 44000)
        duty = 1.00f;
    else if (temperature >= 43000)
        duty = 0.86f;
    else if (temperature >= 42000)
        duty = 0.75f;
    else if (temperature >= 41000)
        duty = 0.58f;
    else if (temperature >= 40000)
        duty = 0.29f;

    CurrentSpeed = FanMax * duty;
    REG_TCC0_CCB3 = int(CurrentSpeed);
    while(TCC0->SYNCBUSY.bit.CCB3);
}

void loop() {
    memset (readData, 0, 256 * sizeof(char));
    char *data_pointer = readData;
    int pending = 0;
    int len = -1;
    unsigned long time;

    // waiting for command to arrive
    time = millis();
    while (Serial.available() <= 0 && (millis() - time) < 30001)
        delay(100);

    // disconnect if no communication
    if (Serial.available() <= 0 && connected) {
        disconnect();
        REG_TCC0_CCB3 = CurrentSpeed = FanMax;
        while(TCC0->SYNCBUSY.bit.CCB3);
        return;
    }

    len = Serial.read();
    if (len <= 1) {
        failed_commands++;
        return; // invalid command
    }

    time = millis();
    pending = len;
    while (pending > 0 && (millis() - time) < 1001) {
        if (!(Serial.available() > 0)) {
            delay(10);
            continue;
        }
        int incomingByte = Serial.read();
        if (incomingByte >= 0) {
            *data_pointer = (char)incomingByte;
            data_pointer++;
            pending--;
        }
    }

    if (pending > 0) {
        failed_commands++;
        reject_msg();
        return;
    }

    if (failed_commands > 9) {
        disconnect();
        return;
    }

    // assembling command code
    unsigned int cmd = readData[0];
    cmd <<= 8;
    cmd += readData[1];

    switch (cmd) {
        case 0x4849: { // HI
            failed_commands = 0;
            connect();
        } return;
        case 0x4242: { // BB
            failed_commands = 0;
            disconnect();
        } return;
        case 0x544d: { // TM
            failed_commands = 0;
            if (len < 6) // not enough data passed
                break;

            long temperature = parse_temperature((unsigned char *)(readData + 2));
            set_fan_speed(temperature);

            Serial.write(0x03);
            Serial.write(0x53);
            Serial.write(0x50);
        Serial.write(int(CurrentSpeed * 100.0f / FanMax) & 0xff);
        } return;
    }

    failed_commands++;
    reject_msg();
}
