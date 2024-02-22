package frc.robot.util;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.math.Matter;

public final class Constants {

  public static final double ROBOT_MASS = 39.97; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final boolean TUNING_MODE = false;

  public static final class ShooterConstants {

    public static final double P = 5e-5;
    public static final double I = 1e-6;
    public static final double D = 0;
    public static final double Iz = 0;
    public static final double FF = 0.000156;
    public static final double MAX_OUTPUT = 1;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_RPM = 5700;
    public static final double MAX_VELOCITY = 2000;
    public static final double MAX_ACCELERATION = 1500;


    public static final double WHEEL_DIAMETER = 4; // inches

  }

  public static final Mode CURRENT_MODE = Mode.REAL;

  public static enum Mode {
    /**
     * Running on a real robot.
     */
    REAL,

    /**
     * Running a physics simulator.
     */
    SIM,

    /**
     * Replaying from a log file.
     */
    REPLAY
  }

  public static final class PositionConstants {
    public static final Pose3d ORIGIN_TO_BACK_LIMELIGHT = new Pose3d(Units.inchesToMeters(-0.25), Units.inchesToMeters(0.25), Units.inchesToMeters(18.1171875), new Rotation3d(0, 0, Math.PI));

    public static final Translation3d ORIGIN_TO_NAVX = new Translation3d(Units.inchesToMeters(-7), Units.inchesToMeters(0), Units.inchesToMeters(6.5));
  }
  public static final class IntakeConstants {
    public static final double HEIGHT_FROM_GROUND = 4.0; // inches
    public static final double CIRCUMFERENCE = HEIGHT_FROM_GROUND * 2 * Math.PI; // inches
    public static final double P = 5e-5;
    public static final double I = 1e-6;
    public static final double D = 0;
    public static final double Iz = 0;
    public static final double FF = 0.000156;
    public static final double MAX_OUTPUT = 1;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_RPM = 240; // 4 rotations per second is the max. Notes are expensive enough as it is.
    public static final double MAX_VELOCITY = 240; // I don't know what these do but fuck it we ball
    public static final double MAX_ACCELERATION = 120;
    public static final double INTAKE_SPEED = 5;
    public static final double CEREALIZER_SPEED = 5;
  }

  public static final class AutonomousConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.04, 0, 0.075);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double JOYSTICK_X_DEADBAND = 0.1;
    public static final double JOYSTICK_Y_DEADBAND = 0.1;
    public static final double JOYSTICK_TWIST_DEADBAND = 0.1;

    public static final double TURN_CONSTANT = 6;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
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
}