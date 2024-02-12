package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class ShooterSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this ShooterSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  private CANSparkFlex lowerMotor, upperMotor;

  // auto log the velocity of the shooter in rpm
  @AutoLogOutput(key = "Shooter/Velocity")
  public double getVelocity() {
    return lowerMotor.getEncoder().getVelocity();
  }


  /**
   * Returns the Singleton instance of this ShooterSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * ShooterSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static ShooterSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this ShooterSubsystem. This constructor is private since this class
   * is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private ShooterSubsystem() {
    lowerMotor = new CANSparkFlex(Constants.MotorConstants.LOWER_SHOOTER_MOTOR,kBrushless);
    upperMotor = new CANSparkFlex(Constants.MotorConstants.UPPER_SHOOTER_MOTOR, kBrushless);
    upperMotor.follow(lowerMotor, true);

    // Set the motors to coast mode
    lowerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    upperMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void shoot() {
    lowerMotor.set(Constants.MotorConstants.SHOOTER_SPEED);
  }

  public void stop() {
    lowerMotor.set(0);
  }
}
