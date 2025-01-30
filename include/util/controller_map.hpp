//
// Created by foamstein on 1/26/25.
//

#ifndef CONTROLLER_MAP_HPP
#define CONTROLLER_MAP_HPP

namespace XBoxController {
    enum class Axis:int {
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 3,
        RIGHT_Y = 4,
        LEFT_TRIGGER = 2,
        RIGHT_TRIGGER = 5
    };

    enum class Button:int {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        BACK = 4,
        GUIDE = 5,
        START = 6,
        LEFT_STICK = 7,
        RIGHT_STICK = 8,
        LEFT_SHOULDER = 9,
        RIGHT_SHOULDER = 10,
        DPAD_UP = 11,
        DPAD_DOWN = 12,
        DPAD_LEFT = 13,
        DPAD_RIGHT = 14,
    };
}

#endif //CONTROLLER_MAP_HPP
