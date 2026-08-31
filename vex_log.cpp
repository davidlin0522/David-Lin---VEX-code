#include "vex_log.h"
#include "vex.h"
#include <string>
#include <iostream>
using namespace vex;
using std::cout;
using std::endl;

int line_num = 0;

void vex_log(V_Log_Level level, std::string message){
    if (level >= V_Global_Log_Level) {
        line_num++;

        // Log message to Computer Terminal
        std::cout << message << std::endl;

        // Log message to Brain Screen
        Brain.Screen.newLine();
        Brain.Screen.print("%d :%s", line_num, message.c_str());

        // Log message to Controller Screen for events V_Critical or greater
        if (level >= V_Critical) {
            Controller1.Screen.newLine();
            Controller1.Screen.print("%d :%s", line_num, message.c_str());
        }
    }
}
