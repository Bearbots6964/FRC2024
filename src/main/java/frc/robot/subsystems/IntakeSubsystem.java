package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this IntakeSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

  // A limit switch to determine whether the intake has a note or not.
  DigitalInput limitSwitch = new DigitalInput(0);

  /**
   * Returns the Singleton instance of this IntakeSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * IntakeSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    return INSTANCE;
  }

  public CANSparkMax intakeMotor;
  private IntakeSubsystem() {
   intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.burnFlash();
  }

  public void SuckOut() {
    intakeMotor.set(0.75);
  }

  public void SuckIn() {
    intakeMotor.set(-0.75);
  }

  public void Stop() {
    intakeMotor.set(0);
  }
}
