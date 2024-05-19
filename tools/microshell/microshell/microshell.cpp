// microshell.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "string.h"

extern "C"{
#include "microshell.h"
}

char test[]="heli\x08p\ninfu\ninfo\nver\n";
int _tmain(int argc, _TCHAR* argv[])
{
	ms_init();
	for(int i=0;i<strlen(test);i++){
		ms_proc_char(test[i]);
	}
	return 0;
}

