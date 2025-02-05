// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int LEFT_SIDE_JOYSICK_AXIS = 1;
        public static final int RIGHT_SIDE_JOYSICK_AXIS = 5;
    }

    public static final class DriveTrainConstants {
        public static final double GEAR_RATIO = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
        public static final double COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
        public static final double COUNTS_PER_REVOLUTION = COUNTS_PER_MOTOR_SHAFT_REV * GEAR_RATIO; // 585.0
        public static final double WHEEL_DIAMETER_INCH = 2.3622; // 60 mm
        public static final double TRACK_WIDTH_INCH = 6.1; // 60 mm
        public static final double MAX_SPEED_METERS_PER_SECOND = 0.2;
    }
}
