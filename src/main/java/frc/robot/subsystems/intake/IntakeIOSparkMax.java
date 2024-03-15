package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// apparently staying up late coding leads to deranged comments so that's cool
public class IntakeIOSparkMax implements IntakeIO {
  public static final double MAX_RPM = 5700.0;
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MotorConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax leftRollerMotor = new CANSparkMax(Constants.MotorConstants.LEFT_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightRollerMotor = new CANSparkMax(Constants.MotorConstants.RIGHT_ROLLER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax cerealizerMotor = new CANSparkMax(Constants.MotorConstants.CEREALIZER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);

  public IntakeIOSparkMax() {

    intakeMotor.setCANTimeout(250);
    leftRollerMotor.setCANTimeout(250);
    rightRollerMotor.setCANTimeout(250);
    cerealizerMotor.setCANTimeout(250);

    intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leftRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    rightRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    cerealizerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    intakeMotor.setInverted(true);
    leftRollerMotor.setInverted(false);
    rightRollerMotor.setInverted(true);
    cerealizerMotor.setInverted(false);

    intakeMotor.getPIDController().setFeedbackDevice(intakeMotor.getEncoder());
    leftRollerMotor.getPIDController().setFeedbackDevice(leftRollerMotor.getEncoder());
    rightRollerMotor.getPIDController().setFeedbackDevice(rightRollerMotor.getEncoder());
    cerealizerMotor.getPIDController().setFeedbackDevice(cerealizerMotor.getEncoder());

    intakeMotor.setSmartCurrentLimit(20);
    leftRollerMotor.setSmartCurrentLimit(20);
    rightRollerMotor.setSmartCurrentLimit(20);
    cerealizerMotor.setSmartCurrentLimit(20);

    intakeMotor.getEncoder().setVelocityConversionFactor(1); // 15:1 gear ratio
    leftRollerMotor.getEncoder().setVelocityConversionFactor(1); // 20:1 gear ratio
    rightRollerMotor.getEncoder().setVelocityConversionFactor(1); // 20:1 gear ratio
    cerealizerMotor.getEncoder().setVelocityConversionFactor(1); // 20:1 gear ratio

    intakeMotor.getPIDController().setP(0.001);
    intakeMotor.getPIDController().setI(0);
    intakeMotor.getPIDController().setD(0.003);
    intakeMotor.getPIDController().setIZone(0);
    intakeMotor.getPIDController().setFF(0.001);
    intakeMotor.getPIDController().setOutputRange(-1, 1);
    intakeMotor.getPIDController().setSmartMotionMaxVelocity(1800, 0);
    intakeMotor.getPIDController().setSmartMotionMinOutputVelocity(0, 0);
    intakeMotor.getPIDController().setSmartMotionMaxAccel(1000, 0);


    cerealizerMotor.getPIDController().setP(0.0005);
    cerealizerMotor.getPIDController().setI(0);
    cerealizerMotor.getPIDController().setD(0.005);
    cerealizerMotor.getPIDController().setIZone(0);
    cerealizerMotor.getPIDController().setFF(0);
    cerealizerMotor.getPIDController().setOutputRange(-1, 1);
    cerealizerMotor.getPIDController().setSmartMotionMaxVelocity(5200, 0);
    cerealizerMotor.getPIDController().setSmartMotionMaxAccel(2000, 0);
    cerealizerMotor.getPIDController().setSmartMotionMinOutputVelocity(0, 0);

    intakeMotor.burnFlash();
    leftRollerMotor.burnFlash();
    rightRollerMotor.burnFlash();
    cerealizerMotor.burnFlash();

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePositionDegrees = intakeMotor.getEncoder().getPosition();
    inputs.intakeVelocityRpm = intakeMotor.getEncoder().getVelocity();
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[]{intakeMotor.getOutputCurrent()};

    inputs.cerealizerPositionDegrees = cerealizerMotor.getEncoder().getPosition();
    inputs.cerealizerVelocityRpm = cerealizerMotor.getEncoder().getVelocity();
    inputs.cerealizerAppliedVolts = cerealizerMotor.getAppliedOutput() * cerealizerMotor.getBusVoltage();
    inputs.cerealizerCurrentAmps = new double[]{cerealizerMotor.getOutputCurrent()};
  }

  @Override
  public void set(double intakePercent, double cerealizerPercent) {
    intakeMotor.set(intakePercent);
    leftRollerMotor.set(intakePercent);
    rightRollerMotor.set(intakePercent);
    cerealizerMotor.set(cerealizerPercent);
  }

  @Override
  public void setVelocity(double intakeRpm, double cerealizerRpm) {
    // clamp the RPM to the max RPM
    intakeRpm = MathUtil.clamp(intakeRpm, 0.0, MAX_RPM);
    cerealizerRpm = MathUtil.clamp(cerealizerRpm, 0.0, MAX_RPM);
    // kid named cambrian explosion
    intakeMotor.getPIDController().setReference(intakeRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
    cerealizerMotor.getPIDController().setReference(cerealizerRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
  }

  @Override
  public void setIntakeVelocity(double intakeRpm) {
    intakeMotor.getPIDController().setReference(intakeRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
  }

  @Override
  public void setCerealizerVelocity(double cerealizerRpm) {
    cerealizerMotor.getPIDController().setReference(cerealizerRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
  }

  @Override
  public void setCerealizer(double a) {
    cerealizerMotor.set(a);
  }

  @Override
  public void setIntake(double a) {
    intakeMotor.set(a);
  }


  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setCerealizerVoltage(double volts) {
    cerealizerMotor.setVoltage(volts);
  }


  public DoubleSupplier getColorSensorProximity() {
    return colorSensor::getProximity;
  }

  public DoubleSupplier getColorSensorRed() {
    return colorSensor::getRed;
  }

  public DoubleSupplier getColorSensorGreen() {
    return colorSensor::getGreen;
  }

  public DoubleSupplier getColorSensorBlue() {
    return colorSensor::getBlue;
  }

  public DoubleSupplier getColorSensorIR() {
    return colorSensor::getIR;
  }

  public Supplier<Color> getColorSensorColor() {
    return colorSensor::getColor;
  }
}
