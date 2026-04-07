#pragma once

#include "config.h"

/* Single-channel commands */
void cmd_read();
void cmd_range(const char* arg);
void cmd_inttime(const char* arg);
void cmd_deadtime(const char* arg);
void cmd_skipvneg(const char* arg1, const char* arg2);
void cmd_clk(const char* arg);
void cmd_test(const char* arg);
void cmd_continuous(const char* arg1, const char* arg2);
void cmd_stream(const char* arg1);
void cmd_capture(const char* arg1);

/* HV switch commands */
void cmd_sw(const char* arg1, const char* arg2);
void cmd_sw2(const char* arg1, const char* arg2);

/* Matrix commands */
void cmd_mscan();
void cmd_mstream();
void cmd_mf32stream();
void cmd_mcapture(const char* arg1);
void cmd_mdscan(const char* arg1);
void cmd_mdstream(const char* arg1);
void cmd_mdcapture(const char* arg1, const char* arg2);
void cmd_md12scan(const char* arg1);
void cmd_md12stream(const char* arg1);
void cmd_md12capture(const char* arg1, const char* arg2);

/* Settings and help */
void cmd_settings();
void cmd_help();
