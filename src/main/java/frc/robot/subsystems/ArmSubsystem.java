package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  private CANSparkMax leftMotor; // Lead motor
  private CANSparkMax rightMotor; // Follow Motor

  /**
   * The Singleton instance of this ShooterSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final ArmSubsystem INSTANCE = new ArmSubsystem();

  /**
   * Returns the Singleton instance of this ShooterSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * ShooterSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static ArmSubsystem getInstance() {
    return INSTANCE;
  }

  // Raises the arm (the right arm is set to mirror the left arm)
  public void raiseArm() {
    leftMotor.set(0.5);
  }

  // Lowers the arm ()
  public void lowerArm() {
    leftMotor.set(-0.5);
  }

  /**
   * Creates a new instance of this ShooterSubsystem. This constructor is private since this class
   * is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private ArmSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    rightMotor = new CANSparkMax(1, MotorType.kBrushless);

    // The right motor will mirror the leader, but it will be inverted
    rightMotor.follow(leftMotor, true);
    
  }
}
