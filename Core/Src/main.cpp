#include "main.h"
#include "BCU/Topology/Topology.hpp"
#include "BCU/Controller/Controller.hpp"
using namespace ST_LIB;

using namespace BCU;
int main(void) {
    Controller::init();
    while (1) {
        Controller::update();
    }
}

extern "C" void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
