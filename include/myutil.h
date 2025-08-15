#ifndef MYUTIL_H
	#define MYUTIL_H
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

void strfcat(char *dst, const char* in, const char* fmt, ...){
	char buf[512];

	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(dst), fmt, args);
	va_end(args);

	strncat(in, buf, 100);
}


#endif // MYUTIL_H
