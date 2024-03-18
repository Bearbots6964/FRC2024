package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this ClimberSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final ClimberSubsystem INSTANCE = new ClimberSubsystem();

  /**
   * Returns the Singleton instance of this ClimberSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * ClimberSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static ClimberSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this ClimberSubsystem. This constructor is private since this class
   * is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private ClimberSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }
}
