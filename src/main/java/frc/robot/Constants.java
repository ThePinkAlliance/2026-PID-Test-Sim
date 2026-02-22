// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Operator Constants
  public static class OperatorConstants {
    // Joystick port
    public static final int kDriverControllerPort = 0;
  }


   public static class ArmSubsystemConstants {
    // Direction enum for arm pivot movement
    public enum DIRECTION {
      FORWARD,
      REVERSE
    }

    // Arm Subsystem Constants
    public static final int ARM_PIVOT_MOTOR_CAN_ID = 21;
    public static final int ARM_IMU_CAN_ID = 0;
    public static final int ARM_STARTSWITCH_DIO_ID = 2; // DIO port on roborio for start limit switch
    public static final int ARM_ENDSWITCH_DIO_ID = 3; // DIO port on roborio for end limit switch
    public static final int ARM_ENCODER_DIO_A = 0; //DIO port on roborio for encoder channel A
    public static final int ARM_ENCODER_DIO_B = 1; //DIO port on roborio for encoder channel B
    public static final int ARM_ENCODER_CPR = 2048; // Counts per revolution for the encoder
    public static final int ARM_ENCODER_SIM_AT_END_POSITION = 1300; // Encoder ticks at the end position for simulation
    public static final int ARM_ENCODER_SIM_AT_START_POSITION = 0; // Encoder ticks at the start position for simulation
    public static final int PIVOT_MOTOR_CURRENT_LIMIT = 60; // Current limit for the pivot motor in amps
    public static final double GOVERNOR_PERCENT = 0.3; // Mentor sugggested governor percentage to limit max speed for safety
    public static final double ARM_MAX_OPERATION_TIME_SECONDS = 4;
    public static final double ARM_MAX_OPERATION_SPEED_POWER = 1.0; // -1.0 to 1.0
    public static final double ARM_PIVOT_PID_TOLERANCE = 5;
    public static final double ARM_PIVOT_PID_MAX_OUTPUT = 0.25;
    public static final double ARM_PIVOT_PID_MIN_OUTPUT = -0.25;
    public static final double ARM_PIVOT_PID_SET_POINT_VERTICAL = 600.0;
    public static final double ARM_PIVOT_PID_SET_POINT_DEPLOYED = 1185.0;
    public static final double ARM_PIVOT_PID_SET_POINT_STOWED = 0.0;
    
    // Returns a safe pivot speed based on a governor constant
    public static double GetSafePivotSpeed(double speed) {
      return speed * GOVERNOR_PERCENT;
    }

  }

  public static class SpinSubsystemConstants {
    public static final int SPIN_SERVO_PWM_ID = 0;
    public static final double SPIN_SERVO_MIN_POSITION = 1.0; // Fully deployed position
    public static final double SPIN_SERVO_MAX_POSITION = 0.0; // Fully stowed position
    public static final double SPIN_SERVO_DEPLOYED_ANGLE = 90.0; // Angle corresponding to fully deployed position
    public static final double SPIN_SERVO_MIN_ANGLE = 0.0; // Angle corresponding to fully stowed position
    public static final double SPIN_SERVO_MAX_ANGLE = 180.0; // Maximum angle the servo can rotate to
  }
}
