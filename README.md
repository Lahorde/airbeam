# Logging library

## Overview
Logging library, supports a variable list of arguments and its format specifiers. 
Log level can be modified according to user needs. 
Using preprocessor logging macros defined in logger.h reduces code size depending on selected logger level.

## Supported targets

  * Arduino compatible (logger needs Stream, Serial...)

## Log levels
 * LOG_LEVEL_NOOUTPUT 
 * LOG_LEVEL_ERRORS 
 * LOG_LEVEL_INFOS 
 * LOG_LEVEL_DEBUG 
 * LOG_LEVEL_VERBOSE 

## Code snippet
Log using a software serial stream.

Define log level according to your needs in logger_config.h :

    /** Global defined must be defined here  */
    #define LOG_LEVEL LOG_LEVEL_DEBUG

Initialize logger with an initialized stream :

    #include <logger_config.h>
    #include <AltSoftSerial.h>
    
    static AltSoftSerial altSerial;

    void setup()
    {
        Serial.begin(115200);
        altSerial.begin(9600);
        
        LOG_INIT(&altSerial);
        LOG_INFO(F("application started ! \n"));
    }
