package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class ArmSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  private final CANSparkMax leftMotor; // Lead motor
  private final CANSparkMax rightMotor; // Follow Motor

  private final SparkPIDController pidController;
  private final SparkAbsoluteEncoder encoder;
  private final double kP;
  private final double kI;
  private final double kD;
  private final double kIz;
  private final double kFF;
  private final double kMaxOutput;
  private final double kMinOutput; // We'll probably want to put these in the constants class later
  private double maxRange; // amount of revolutions (or other unit) it takes to move the arm from min to max


  /**
   * The Singleton instance of this ArmSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final ArmSubsystem INSTANCE = new ArmSubsystem();

  /**
   * Returns the Singleton instance of this ArmSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * ArmSubsystem.getInstance();}
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

  // Control the arm via voltage
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
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
   * Creates a new instance of this ArmSubsystem. This constructor is private since this class
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
  SysIdRoutine armSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, null, null,
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())
      ),
      new SysIdRoutine.Mechanism((v) -> this.setVoltage(v.in(Volts)), null, this)
  );

  public SysIdRoutine getSysId() {
    return armSysId;
  }

  public static Command generateSysIdCommand(SysIdRoutine sysIdRoutine, double delay, double quasiTimeout,
                                             double dynamicTimeout)
  {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
        .andThen(Commands.waitSeconds(delay))
        .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
        .andThen(Commands.waitSeconds(delay))
        .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout));
  }
}
