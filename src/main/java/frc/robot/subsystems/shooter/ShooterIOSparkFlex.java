package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import frc.robot.util.Constants;

public class ShooterIOSparkFlex implements ShooterIO {
  private final CANSparkFlex lowerMotor = new CANSparkFlex(Constants.MotorConstants.LOWER_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex upperMotor = new CANSparkFlex(Constants.MotorConstants.UPPER_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  private final RelativeEncoder upperEncoder = upperMotor.getEncoder();
  private final SparkPIDController lowerPID = lowerMotor.getPIDController();
  private final SparkPIDController upperPID = upperMotor.getPIDController();
  public static final double P = Constants.ShooterConstants.P;
  public static final double I = Constants.ShooterConstants.I;
  public static final double D = Constants.ShooterConstants.D;


  public ShooterIOSparkFlex() {
//
//    // Set CAN timeouts
//    lowerMotor.setCANTimeout(250);
//    upperMotor.setCANTimeout(250);
//
//    // Set motor inversions
//    lowerMotor.setInverted(false);
//    upperMotor.setInverted(true);
//
//    // Enable voltage compensation
//    lowerMotor.enableVoltageCompensation(12.0);
//    upperMotor.enableVoltageCompensation(12.0);
//
//    lowerMotor.getPIDController().setFeedbackDevice(lowerEncoder);
//    upperMotor.getPIDController().setFeedbackDevice(upperEncoder);
//
//    // Set smart current limits
//    lowerMotor.setSmartCurrentLimit(40);
//    upperMotor.setSmartCurrentLimit(40);
//
//    // Set PID constants
//    lowerPID.setP(P);
//    lowerPID.setI(I);
//    lowerPID.setD(D);
//    lowerPID.setFF(Constants.ShooterConstants.FF);
//    upperPID.setP(P);
//    upperPID.setI(I);
//    upperPID.setD(D);
//    upperPID.setFF(Constants.ShooterConstants.FF);
//
//    lowerPID.setSmartMotionMaxVelocity(5700, 0);
//    lowerPID.setSmartMotionMinOutputVelocity(0, 0);
//    upperPID.setSmartMotionMaxVelocity(5700, 0);
//    upperPID.setSmartMotionMinOutputVelocity(0, 0);
//
//    // Burn flash
//    lowerMotor.burnFlash();
//    upperMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.lowerPositionDegrees = lowerEncoder.getPosition();
    inputs.lowerVelocityRpm = lowerEncoder.getVelocity();
    inputs.lowerAppliedVolts = lowerMotor.getAppliedOutput() * lowerMotor.getBusVoltage();
    // Theoretically,
    // I could implement a method to set surface speed and have that be the value the motors try to get to,
    // but I don't know if we would actually use that.
    // TODO: determine if we want to use surface speed
    inputs.lowerSurfaceSpeed = lowerEncoder.getVelocity() * Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI / 60.0 / 12.0; // feet per second
    inputs.lowerOutput = lowerMotor.get();
    inputs.lowerCurrentAmps = new double[] {lowerMotor.getOutputCurrent()};

    inputs.upperPositionDegrees = upperEncoder.getPosition();
    inputs.upperVelocityRpm = upperEncoder.getVelocity();
    inputs.upperAppliedVolts = upperMotor.getAppliedOutput() * upperMotor.getBusVoltage();
    inputs.upperSurfaceSpeed = upperEncoder.getVelocity() * Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI / 60.0 / 12.0; // feet per second
    inputs.upperOutput = upperMotor.get();
    inputs.upperCurrentAmps = new double[] {upperMotor.getOutputCurrent()};
  }

  @Override
  public void set(double lowerPercent, double upperPercent) {
    lowerMotor.set(lowerPercent);
    upperMotor.set(upperPercent);
  }

  @Override
  public void setVelocity(double lowerRpm, double upperRpm) {
    lowerPID.setReference(lowerRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
    upperPID.setReference(upperRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
  }

}
