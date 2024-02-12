package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** A limit switch to determine whether the intake has a note or not. */
  DigitalInput limitSwitch = new DigitalInput(0);
  public CANSparkMax intakeMotor;

  private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

  private IntakeSubsystem() {
   intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.burnFlash();
  }

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    return INSTANCE;
  }

  public void suckOut() {
    intakeMotor.set(0.75);
  }

  public void suckIn() {
    intakeMotor.set(-0.75);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}
