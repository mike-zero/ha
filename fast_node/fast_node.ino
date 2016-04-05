/* fast measuring sensor node */

#define LED 13

#define analog_pins 8
#define digital_pins 14

// for analog_data and digital_data
#define DATA_LAST 0
#define DATA_CHANGES 1
#define DATA_MIN 2
#define DATA_MAX 3

#define ANALOG_DATA_STATES 4	// last, changes, min, max
#define DIGITAL_DATA_STATES 2	// last, changes

// for analog_data_big
#define DATA_SUM 0
#define ANALOG_DATA_BIG_STATES 1

uint16_t report_period_millis = 5 * 1000;
uint16_t fast_measures_per_report = 1000;
uint16_t slow_measures_per_report = 2;
unsigned long next_report_time = 0;
unsigned long next_fast_measure = 0;
unsigned long next_slow_measure = 0;

unsigned long fast_measure_period = fast_measures_per_report > 0 ? report_period_millis / fast_measures_per_report : 0;
unsigned long slow_measure_period = slow_measures_per_report > 0 ? report_period_millis / slow_measures_per_report : 0;
unsigned long current_millis;
signed long wait_millis;
signed long wait_millis_temp;

uint16_t analog_mask = 0x0003;
uint16_t analog_data[analog_pins][ANALOG_DATA_STATES];
unsigned long analog_data_big[analog_pins][1];

uint16_t digital_mask = 0x0004;
uint16_t digital_data[digital_pins][DIGITAL_DATA_STATES];

uint16_t mask = 0;
uint8_t pin = 0;
uint16_t count_fast = 0;
uint16_t count_slow = 0;

void setup() {
	memset(analog_data, 0, sizeof(analog_data));
	memset(analog_data_big, 0, sizeof(analog_data_big));
	Serial.begin(115200);
	Serial.print(millis());
	Serial.println(" Start");
#ifdef LED
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);
#endif
}

void loop() {
	current_millis = millis();
	wait_millis = report_period_millis;	// cannot be more than the report period
	if (fast_measures_per_report > 0) {
		wait_millis_temp = (signed long)(next_fast_measure - current_millis);
		if (wait_millis_temp <= 0) {
			fast_measure();
			return;
		} else {
			if (wait_millis_temp < wait_millis) wait_millis = wait_millis_temp;
		}
	}
	if (slow_measures_per_report > 0) {
		wait_millis_temp = (signed long)(next_slow_measure - current_millis);
		if (wait_millis_temp <= 0) {
			slow_measure();
			return;
		} else {
			if (wait_millis_temp < wait_millis) wait_millis = wait_millis_temp;
		}
	}
	wait_millis_temp = (signed long)(next_report_time - current_millis);
	if (wait_millis_temp <= 0) {
		report();
		return;
	} else {
		if (wait_millis_temp < wait_millis) wait_millis = wait_millis_temp;
	}
#ifdef LED
	digitalWrite(LED, LOW);
#endif
	delay(millis() - current_millis + wait_millis);
#ifdef LED
	digitalWrite(LED, HIGH);
#endif
}

void report() {
	next_report_time += report_period_millis;
	Serial.print(" count_fast=");
	Serial.print(count_fast);
	Serial.print(" count_slow=");
	Serial.println(count_slow);

	mask = analog_mask;
	pin = 0;
	while (mask > 0) {
		if ((mask & 1) == 1) {
			Serial.print(" A");
			Serial.print(pin);
			Serial.print(": ");
			Serial.print(analog_data[pin][DATA_MIN]);
			Serial.print("/");
			Serial.print(analog_data[pin][DATA_LAST]);
			Serial.print("/");
			Serial.print(analog_data[pin][DATA_MAX]);
			Serial.print(" sum: ");
			Serial.print(analog_data_big[pin][DATA_SUM]);
			Serial.print(" chgs: ");
			Serial.print(analog_data[pin][DATA_CHANGES]);
			// just for debug
			Serial.print(" avg: ");
			Serial.print(analog_data_big[pin][DATA_SUM] / count_fast);

			analog_data[pin][DATA_MIN] = 0;
			analog_data[pin][DATA_MAX] = 0;
			analog_data[pin][DATA_CHANGES] = 0;
			analog_data_big[pin][DATA_SUM] = 0;
			Serial.println();
		}
		pin++;
		mask >>= 1;
	}

	mask = digital_mask;
	pin = 0;
	while (mask > 0) {
		if ((mask & 1) == 1) {
			Serial.print(" D");
			Serial.print(pin);
			Serial.print(": ");
			Serial.print(digital_data[pin][DATA_LAST]);
			Serial.print(" chgs: ");
			Serial.print(digital_data[pin][DATA_CHANGES]);

			digital_data[pin][DATA_CHANGES] = 0;
			Serial.println();
		}
		pin++;
		mask >>= 1;
	}

	Serial.println();
	count_fast = 0;
	count_slow = 0;
}

void fast_measure() {
	next_fast_measure += fast_measure_period;
//	Serial.print(millis());
//	Serial.println(" Fast");
	mask = analog_mask;
	uint8_t pin = 0;
	while (mask > 0) {
		if ((mask & 1) == 1) {
			int val = analogRead(pin);
			if (val != analog_data[pin][DATA_LAST]) analog_data[pin][DATA_CHANGES]++;
			analog_data_big[pin][DATA_SUM] += val;
			if (count_fast == 0 || val < analog_data[pin][DATA_MIN]) analog_data[pin][DATA_MIN] = val;
			if (count_fast == 0 || val > analog_data[pin][DATA_MAX]) analog_data[pin][DATA_MAX] = val;
			analog_data[pin][DATA_LAST] = val;
		}
		pin++;
		mask >>= 1;
	}
	mask = digital_mask;
	pin = 0;
	while (mask > 0) {
		if ((mask & 1) == 1) {
			int val = digitalRead(pin);
			if (val != digital_data[pin][DATA_LAST]) digital_data[pin][DATA_CHANGES]++;
			digital_data[pin][DATA_LAST] = val;
		}
		pin++;
		mask >>= 1;
	}
	count_fast++;
}

void slow_measure() {
	// TODO: add measure phases, remember the start_time, wake_up for every phase and advance to the next
	next_slow_measure += slow_measure_period;
//	Serial.print(millis());
//	Serial.println(" Slow");
	count_slow++;
}
