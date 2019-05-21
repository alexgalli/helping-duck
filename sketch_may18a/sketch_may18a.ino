const int PIEZO_PIN = 8;
const int NOTE_LENGTH = 1000;
const unsigned int FREQ_INIT = 100;
const unsigned int FREQ_MAX = 8000;
const double FREQ_FACTOR = 1.8;

volatile unsigned int freq = FREQ_INIT;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.write("starting up...\n");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  /*
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  Serial.write("hello\n");
  */

  printFreq(freq, "playing");
  
  playNote(freq);
  
  if (freq <= FREQ_MAX) {
    freq = min(int(freq * FREQ_FACTOR), FREQ_MAX + 1);
  }
  else
    freq = FREQ_INIT;

  /*
  Serial.write("freq: ");
  Serial.write(freq);
  Serial.write(".\n");
  */
}

void playNote(unsigned int f) {
  tone(PIEZO_PIN, f, NOTE_LENGTH);
  delay(NOTE_LENGTH);
}

void printFreq(unsigned int f, const char *prefix) {
  static char buffer[20];
  sprintf(buffer, "%s: freq: %d\n", prefix, f);
  Serial.write(buffer);
}
