package frc.robot.util;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonomousConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class MotorConstants {

    // Shooter Motor Constants
    public static final int LOWER_SHOOTER_MOTOR = 13;
    public static final int UPPER_SHOOTER_MOTOR = 12;
    public static final double SHOOTER_SPEED = 0.70;
    // Arm Motor Constants
    public static final int LEFT_ARM_MOTOR = 14; // Currently the ids are arbitrary.
    public static final int RIGHT_ARM_MOTOR = 15; // They will need to be adjusted to reflect the actual connected ports
    // Intake Constants
    public static final int INTAKE_MOTOR = 10;
    public static final int CEREALIZER_MOTOR = 11;
    public static final double INTAKE_SPEED = 0.45;
  }

  public static class ArmMotorConstants {

    // Arm Motor Constants
    public static final int LEFT_ARM_MOTOR = 14; // Currently the ids are arbitruary. 
    public static final int RIGHT_ARM_MOTOR = 15; // They will need to be adjusted to reflect the actual connected ports
    public static final double SHOOTER_SPEED = 0.70;
  }
}