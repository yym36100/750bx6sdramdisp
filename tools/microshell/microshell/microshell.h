#pragma once


#define MS_FONT_STYLE_RESET             "\x1B[0m"
#define MS_FONT_STYLE_BOLD              "\x1B[1m"
#define MS_FONT_STYLE_DISABLED          "\x1B[2m"
#define MS_FONT_STYLE_ITALIC            "\x1B[3m"
#define MS_FONT_STYLE_UNDERSCORE        "\x1B[4m"

#define MS_FONT_COLOR_RED               "\x1B[31m"
#define MS_FONT_COLOR_GREEN             "\x1B[32m"
#define MS_FONT_COLOR_YELLOW            "\x1B[33m"
#define MS_FONT_COLOR_BLUE              "\x1B[34m"
#define MS_FONT_COLOR_MAGENTA           "\x1B[35m"
#define MS_FONT_COLOR_CYAN              "\x1B[36m"
#define MS_FONT_COLOR_WHITE             "\x1B[37m"


typedef struct ms_tcmd{
	char *name;
	char *descr;
	void (*func)(int argc, void* argv[]);	
}ms_tcmd;

#define ms_hist_size	(10)
typedef struct  ms_tRingBuff {	
	char items[ms_hist_size][32];
	int rp, wp;
}ms_tRingBuff;



typedef struct ms_tctx{
	int index;
	char line_buff[255];
}ms_tctx;

void ms_init(void);
void ms_proc_char(char c);

void ms_hist_init(void);

void ms_hist_add(char *t);

void ms_hist_show(void);
