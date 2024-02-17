package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.Constants;

// apparently staying up late coding leads to deranged comments so that's cool
public class IntakeIOSparkMax implements IntakeIO {
  public static final double MAX_RPM = 5700.0;
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.MotorConstants.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax cerealizerMotor = new CANSparkMax(Constants.MotorConstants.CEREALIZER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

  public IntakeIOSparkMax() {
    intakeMotor.restoreFactoryDefaults();
    cerealizerMotor.restoreFactoryDefaults();

    intakeMotor.setCANTimeout(250);
    cerealizerMotor.setCANTimeout(250);

    intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    cerealizerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    intakeMotor.setInverted(false);
    cerealizerMotor.setInverted(true);

    intakeMotor.enableVoltageCompensation(12.0);
    cerealizerMotor.enableVoltageCompensation(12.0);

    intakeMotor.setSmartCurrentLimit(20);
    cerealizerMotor.setSmartCurrentLimit(20);

    intakeMotor.getEncoder().setVelocityConversionFactor((double) 1 / 15); // 15:1 gear ratio
    cerealizerMotor.getEncoder().setVelocityConversionFactor((double) 1 / 20); // 20:1 gear ratio

    intakeMotor.getPIDController().setP(Constants.IntakeConstants.P);
    intakeMotor.getPIDController().setI(Constants.IntakeConstants.I);
    intakeMotor.getPIDController().setD(Constants.IntakeConstants.D);
    intakeMotor.getPIDController().setIZone(Constants.IntakeConstants.Iz);
    intakeMotor.getPIDController().setFF(Constants.IntakeConstants.FF);
    intakeMotor.getPIDController().setOutputRange(Constants.IntakeConstants.MIN_OUTPUT, Constants.IntakeConstants.MAX_OUTPUT);

    intakeMotor.burnFlash();
    cerealizerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePositionDegrees = intakeMotor.getEncoder().getPosition();
    inputs.intakeVelocityRpm = intakeMotor.getEncoder().getVelocity();
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.intakeCurrentAmps = new double[] {intakeMotor.getOutputCurrent()};

    inputs.cerealizerPositionDegrees = cerealizerMotor.getEncoder().getPosition();
    inputs.cerealizerVelocityRpm = cerealizerMotor.getEncoder().getVelocity();
    inputs.cerealizerAppliedVolts = cerealizerMotor.getAppliedOutput() * cerealizerMotor.getBusVoltage();
    inputs.cerealizerCurrentAmps = new double[] {cerealizerMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double intakeVolts, double cerealizerVolts) {
    intakeMotor.setVoltage(intakeVolts);
    cerealizerMotor.setVoltage(cerealizerVolts);
  }

  @Override
  public void setVelocity(double intakeRpm, double cerealizerRpm) {
    // clamp the RPM to the max RPM
    intakeRpm = MathUtil.clamp(intakeRpm, 0, MAX_RPM);
    cerealizerRpm = MathUtil.clamp(cerealizerRpm, 0, MAX_RPM);
    // kid named cambrian explosion
    intakeMotor.getPIDController().setReference(intakeRpm, CANSparkBase.ControlType.kSmartVelocity, 0, Constants.IntakeConstants.FF);
    cerealizerMotor.getPIDController().setReference(cerealizerRpm, CANSparkBase.ControlType.kSmartVelocity, 0, Constants.IntakeConstants.FF);
  }


}
