
#ifndef AUTOGEN_CLARA_MACROS_H
#define AUTOGEN_CLARA_MACROS_H
    #include <string.h>
    //! macro preprocessing of the file standart to get the filename instead of the full filepath
    #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

    #include <iostream>
    //! logging based on the DEBUG_LEVEL_CLARA defined while building with cmake (only active with DEBUG_LEVEL=2)
    #define DEBUG_MSG_CLARA(msg) std::cerr << "[CLARA - " \
                        << __FILENAME__ << ':' \
                        << __LINE__ << ':'     \
                        << __func__ << "()]: "    \
                        << msg;
    //! logging based on the DEBUG_LEVEL_CLARA defined while building with cmake (active with DEBUG_LEVEL=1 and 2)
    #define DEBUG_CRIT_MSG_CLARA(msg) DEBUG_MSG_CLARA(msg)
#endif // AUTOGEN_CLARA_MACROS_H