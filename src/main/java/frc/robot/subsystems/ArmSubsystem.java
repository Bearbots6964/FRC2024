package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class ArmSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  private CANSparkMax leftMotor; // Lead motor
  private CANSparkMax rightMotor; // Follow Motor

  private SparkPIDController pidController;
  private SparkAbsoluteEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput; // We'll probably want to put these in the constants class later
  private double maxRange; // amount of revolutions (or other unit) it takes to move the arm from min to max


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

  // This should set the current arm location to be zero on the encoder?
  // MAKE SURE THAT THE ZERO OFFSET IS SET WHEN THE ARM IS AT IT'S LOWER LIMIT
  // Set lower limit before upper limit.
  public void setZeroOffset() {
    encoder.setZeroOffset(encoder.getPosition());
  }

  // Sets max range
  public void setMaxRange() {
    maxRange = encoder.getPosition();
  }

  private void rotate(double rotations) {
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  // Takes a double 0 to 1 as a parameter, where 0 is the arm at its lower limit
  // and 1 at its upper limit.
  public void moveArm(double p) {
    double distanceFromMin = p * maxRange;
    double distanceFromFinal = distanceFromMin - encoder.getPosition(); // Final - inital = delta position.
    rotate(distanceFromFinal);
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
    leftMotor = new CANSparkMax(Constants.MotorConstants.LEFT_ARM_MOTOR, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_ARM_MOTOR, MotorType.kBrushless);

    // The right motor will mirror the leader, but it will be inverted
    rightMotor.follow(leftMotor, true);

    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(20);
    leftMotor.burnFlash();

    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(20);
    rightMotor.burnFlash();

    pidController = leftMotor.getPIDController();
    encoder = leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);


    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    
  }
}
