#include "pca.h"

int main(void)
{
	// enable global interrupts
	sei();

	struct soft_serial soft_serial_bus;
	soft_serial_bus.tx.port = &PORTD;
	soft_serial_bus.tx.pin = 2;
	soft_serial_bus.rx.port = &PORTD;
	soft_serial_bus.rx.pin = 3;
	soft_serial_bus.baudrate = E_SSBAUD_9600;

	soft_serial_init(&soft_serial_bus);
	soft_serial_install_stdio();

	while (1) {
		 _delay_ms(500);

		unsigned char size = soft_serial_available();
		unsigned char c[64] = { 0 };

		if (size > 0) {
			soft_serial_recv(&c[0], size, 0);
			TRACE_MSG("%s\n", &c[0]);
		}

		printf("Software Serial\n");
	}

	return 0;
}
