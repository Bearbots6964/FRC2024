package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
  /**
   * A limit switch to determine whether the intake has a note or not.
   */
  // DigitalInput limitSwitch = new DigitalInput(0);
  public CANSparkMax intakeMotor, cerealizerMotor;

  private IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.MotorConstants.INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.burnFlash();

    cerealizerMotor = new CANSparkMax(Constants.MotorConstants.CEREALIZER_MOTOR, MotorType.kBrushless);
    cerealizerMotor.setIdleMode(IdleMode.kBrake);
    cerealizerMotor.setSmartCurrentLimit(20);
    cerealizerMotor.burnFlash();
  }

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    return INSTANCE;
  }

  public void suckOut() {
    intakeMotor.set(Constants.MotorConstants.INTAKE_SPEED);
    cerealizerMotor.set(Constants.MotorConstants.INTAKE_SPEED);
  }

  public void suckIn() {
    intakeMotor.set(-Constants.MotorConstants.INTAKE_SPEED);
    cerealizerMotor.set(-Constants.MotorConstants.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}
