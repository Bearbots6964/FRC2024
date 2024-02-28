package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;


public class VisionSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this VisionSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final VisionSubsystem INSTANCE = new VisionSubsystem();

  /**
   * Returns the Singleton instance of this VisionSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * VisionSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static VisionSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this VisionSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private VisionSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SwerveSubsystem.addVisionMeasurement(LimelightHelpers.getBotPose2d("back-limelight"), Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture("back-limelight") - LimelightHelpers.getLatency_Pipeline("back-limelight") );

    Logger.recordOutput("Targets", LimelightHelpers.getTargetPose3d_RobotSpace("limelight-back"));

  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the
  // "tx" value from the Limelight.
  public double limelight_aim_proportional_front() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveSubsystem.getInstance().getSwerveDrive().getMaximumAngularVelocity();

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public double limelight_aim_proportional_back() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-back") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveSubsystem.getInstance().getSwerveDrive().getMaximumAngularVelocity();

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= 1.0;

    return targetingAngularVelocity;
  }
  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-front") * kP;
    targetingForwardSpeed *= SwerveSubsystem.getInstance().getSwerveDrive().getMaximumVelocity();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
}
