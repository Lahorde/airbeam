#ifndef LOGGER_CONFIG_H
#define LOGGER_CONFIG_H

/** In arduino we cannot specify any project specific define,
 * this hack defines log level, file needing logs have just to 
 * include it */

/** Global log level must be defined here  */
#define LOG_LEVEL LOG_LEVEL_DEBUG
#include <logger.h>

#endif /** LOGGER_CONFIG_H */
