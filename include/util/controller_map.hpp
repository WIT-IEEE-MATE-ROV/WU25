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
        RIGHT_TRIGGER = 5,
        DPAD_UP_DOWN = 7,
        DPAD_LEFT_RIGHT = 6
    };

    enum class Button:int {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        BACK = 6,
        GUIDE = 8,
        START = 7,
        LEFT_BUMPER = 4,
        RIGHT_BUMPER = 5,
        LEFT_STICK = 9,
        RIGHT_STICK = 10,
//        DPAD_UP = 11,
//        DPAD_DOWN = 12,
//        DPAD_LEFT = 13,
//        DPAD_RIGHT = 14,
    };
}

#endif //CONTROLLER_MAP_HPP
