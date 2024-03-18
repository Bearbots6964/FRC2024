package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SparkLimitSwitch forwardLimitSwitch, reverseLimitSwitch;

  private final SparkPIDController pidController;
  private final SparkAbsoluteEncoder encoder;
  private static final double kP = 0.1;
  private static final double kI = 1e-4;
  private static final double kD = 1;
  private static final double kIz = 0;
  private static final double kFF = 0;
  private static final double kMaxOutput = 1;
  private static final double kMinOutput = -1; // We'll probably want to put these in the constants class later
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

  public void moveArmButBetter(double speed) {
    leftMotor.set(speed);
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

    encoder = rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    encoder.setPositionConversionFactor(360); // 1 rotation = 360 degrees
    // I don't want to have to deal with radians because of rounding errors

    // The right motor will mirror the leader, but it will be inverted
    rightMotor.follow(leftMotor, true);

    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(40);
    leftMotor.burnFlash();

    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(40);
    rightMotor.burnFlash();

    pidController = rightMotor.getPIDController();


    forwardLimitSwitch = rightMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = rightMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    forwardLimitSwitch.enableLimitSwitch(true);
    reverseLimitSwitch.enableLimitSwitch(true);


    encoder.setPositionConversionFactor(360); // 1 rotation = 360 degrees
    // I don't want to have to deal with radians because of rounding errors


    //
//    pidController.setP(kP);
//    pidController.setI(kI);
//    pidController.setD(kD);
//    pidController.setIZone(kIz);
//    pidController.setFF(kFF);
//    pidController.setOutputRange(kMinOutput, kMaxOutput);
    
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


 @Override
 public void periodic() {
   // This method will be called once per scheduler run

   // 0 degrees is set to (r, 270) in polar coordinates, because that way it will never wrap around
   // If the forward limit switch is pressed, we're at 243 degrees
   if(forwardLimitSwitch.isPressed()) {
     encoder.setZeroOffset(242.88780212402344);
   }
   // If the reverse limit switch is pressed, we're at 40 degrees
   if(reverseLimitSwitch.isPressed()) {
     encoder.setZeroOffset(39.87791442871094);
   }

   SmartDashboard.putNumber("Arm Position", encoder.getPosition());
 }
}
