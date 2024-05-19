#include "microshell.h"
#include <stdio.h>

void cmd_help(int argc, void* argv[]){
	printf(MS_FONT_COLOR_GREEN"cmd_help was called"MS_FONT_COLOR_WHITE"\n");
}

#define BUILD_DATE_TIME __DATE__ " " __TIME__

void cmd_ver(int argc, void* argv[]){
	printf("Build date and time: %s\n", BUILD_DATE_TIME);
}

void cmd_hist(int argc, void* argv[]){
	printf("Command history:\n");
	ms_hist_show();
}


ms_tcmd ms_cmds[] = {
	{ "info", "nothing useful", 0 },
	{ "help", "show this text", cmd_help },
	{ "ver" , "built on: t"BUILD_DATE_TIME,cmd_ver },
	{ "hist" , "shows cmd history",cmd_hist },
};

ms_tctx ms_ctx;
ms_tRingBuff ms_hist;

int ms_hist_isEmpty() {return ms_hist.rp==ms_hist.wp;}
int ms_hist_getSize() {
	int delta= ms_hist.wp-ms_hist.rp;
	if(delta==0)return 0;
	if(delta<0) delta += ms_hist_size;
	return delta;
}

int ms_hist_isFull() { return ms_hist_getSize() == ms_hist_size - 1; }

void ms_hist_inc_wp(){ms_hist.wp++;if(ms_hist.wp==ms_hist_size) ms_hist.wp=0;}
void ms_hist_inc_rp(){ms_hist.rp++;if(ms_hist.rp==ms_hist_size) ms_hist.rp=0;}

void ms_hist_init(void){
	ms_hist.rp = ms_hist.wp = 0;
}
void ms_hist_add(char *t){
	// overwrite as needed
	//ms_hist.items[wp] = d;
	strcpy(ms_hist.items[ms_hist.wp] ,t);
	ms_hist_inc_wp();
}
void ms_hist_show(void){
	int sz = ms_hist_getSize();	
	int i;
	for(i=0;i<sz;i++){
		printf("%d: %s\n",i,&ms_hist.items[i][0]);
	}
}


void ms_init(void){
	ms_ctx.index = 0;
}

void ms_proc_char(char c){
	int i=0;
	switch(c){
		default:
			//printf("proc char: %c %02x\n",c,c);
			ms_ctx.line_buff[ms_ctx.index++] = c;
			break;
		case '\n':
		case '\r':
			ms_ctx.line_buff[ms_ctx.index] = 0;
			ms_hist_add(ms_ctx.line_buff);
			printf("cmd rec: %s\n",ms_ctx.line_buff);

			// search for cmd
			for(i=0;i<sizeof(ms_cmds)/sizeof(ms_cmds[0]);i++){
				if(strcmp(ms_cmds[i].name,ms_ctx.line_buff)==0){
					printf("command found: %s\n",ms_cmds[i].descr);
					if(ms_cmds[i].func!=0){
						ms_cmds[i].func(0,0);
					} else {
						printf(MS_FONT_COLOR_RED"command exists but there is no function to call"MS_FONT_COLOR_WHITE"\n");
					}
					ms_ctx.index = 0;
					return;
				}
			}
			printf("command not found\n");
			ms_ctx.index = 0;
			break;
		case 8:
		case 127:
			//printf("backspace handling\n");
			ms_ctx.index--;
			break;
	}

}
