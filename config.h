#ifndef CONFIG_H
#define CONFIG_H

#if __has_include("Config.h")
    #include "Config.h"
#else
#warning "Using default settings: Copy Config_Default.h to Config.h and edit it to fit your needs."
    #include "Config_Default.h"
#endif

#endif