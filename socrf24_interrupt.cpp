#include <stdbool.h>
#include <stdio.h>

extern "C" {

int socrf24_irq_handler(void *arg)
{
	fprintf(stderr, "IRQ\n");
	volatile bool *flg = (volatile bool *)arg;
	*flg = true;
	return 0;
}

};
