#define COMMAND_MAX_LEN 256
char readData[COMMAND_MAX_LEN];
int failed_commands = 0;
bool connected = false;

int FanMax = 28;
float CurrentSpeed = 0;

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
    OCR2B = int(CurrentSpeed);
}

void setup() {
    // generate 25kHz PWM pulse rate on Pin 3
    pinMode(3, OUTPUT);   // OCR2B sets duty cycle
    // Set up Fast PWM on Pin 3
    TCCR2A = 0x23;     // COM2B1, WGM21, WGM20
    // Set prescaler
    TCCR2B = 0x0A;   // WGM21, Prescaler = /8
    // Set TOP and initialize duty cycle to zero(0)
    OCR2A = 79;    // TOP DO NOT CHANGE, SETS PWM PULSE RATE
    OCR2B = int(CurrentSpeed);

    Serial.begin(9600);
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
        OCR2B = CurrentSpeed = FanMax;
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

            long temperature = parse_temperature(readData + 2);
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
