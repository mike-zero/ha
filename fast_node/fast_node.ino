/* fast measuring sensor node */

uint16_t report_period_millis = 60 * 1000;
uint16_t fast_measures_per_report = 60;
uint16_t slow_measures_per_report = 12;
unsigned long next_report_time = 0;
unsigned long next_fast_measure = 0;
unsigned long next_slow_measure = 0;

unsigned long fast_measure_period = fast_measures_per_report > 0 ? report_period_millis / fast_measures_per_report : 0;
unsigned long slow_measure_period = slow_measures_per_report > 0 ? report_period_millis / slow_measures_per_report : 0;

void setup() {
	Serial.begin(115200);
	Serial.print(millis());
	Serial.println("Start");
}

void loop() {
	if (fast_measures_per_report > 0 && ((signed long)(millis() - next_fast_measure)) >= 0) fast_measure();
	if (slow_measures_per_report > 0 && ((signed long)(millis() - next_slow_measure)) >= 0) slow_measure();
	if (((signed long)(millis() - next_report_time)) >= 0) report();
}

void report() {
	next_report_time = millis() + report_period_millis;
	Serial.print(millis());
	Serial.println(" Report");
}

void fast_measure() {
	next_fast_measure = millis() + fast_measure_period;
	Serial.print(millis());
	Serial.println(" Fast");
}

void slow_measure() {
	// TODO: add measure phases, remember the start_time, wake_up for every phase and advance to the next
	next_slow_measure = millis() + slow_measure_period;
	Serial.print(millis());
	Serial.println(" Slow");
}
