#ifndef __KERNEL_UTILS_H_
#define __KERNEL_UTILS_H_

#include <termios.h>
#include <fcntl.h>
#include <stddef.h>

#define container_of(ptr, type, member)					\
	({								\
		const typeof(((type *)0)->member) *__mptr = (ptr);	\
		(type *)((char *)__mptr - offsetof(type, member));	\
	})

#define STD_IN		0
static
inline char kbhit_irq(void)
{
	struct termios oldt, newt;
	int maxfd = 0;
	fd_set fds;
	char c[3];

	for(;;) {

		FD_ZERO(&fds);
		FD_SET(STD_IN, &fds);

		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);

		select(maxfd + 1, &fds, NULL, NULL, NULL);
		read(STD_IN, &c, 3);

		/* if it is an ascii escape sequence, anything different than
		 *  the arrow keys will terminate the calibration
		 */
		if (c[0] == 0x1b && c[1] == 0x5b) {
			switch(c[2]) {
			case 'A':
			case 'B':
			case 'C':
			case 'D':
				goto done;
			}
		} else {
			c[2] = 'X';
			break;
		}
	}
done:
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return c[2];
}

#endif
