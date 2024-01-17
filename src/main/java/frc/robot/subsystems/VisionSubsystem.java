package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
