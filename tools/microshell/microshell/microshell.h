#pragma once

typedef struct ms_tcmd{
	char *name;
	char *descr;
	void (*func)(int argc, void* argv[]);	
}ms_tcmd;

typedef struct ms_tctx{
	int index;
	char line_buff[255];
}ms_tctx;

void ms_init(void);
void ms_proc_char(char c);