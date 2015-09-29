#include <stdbool.h>

extern "C" {

int socrf24_irq_handler(void *arg)
{
	volatile bool *flg = (volatile bool *)arg;
	*flg = true;
	return 0;
}

};
