//-- includes -----
#include "ClientLog.h"

//-- globals -----
e_log_severity_level g_min_log_level;

// Connect the normal logger to standard output
std::ostream g_normal_logger(std::cout.rdbuf());

// Null stream eats the log
NullStream<char> g_null_logger;

//-- public implementation -----
void log_init(e_log_severity_level min_log_level)
{
    g_min_log_level= min_log_level;
}

bool log_can_emit_level(e_log_severity_level level)
{
    return (level >= g_min_log_level);
}
