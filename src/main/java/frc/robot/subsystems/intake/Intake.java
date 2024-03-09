package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import org.littletonrobotics.junction.Logger;
// Not doing the sim because I would like to retain what little sanity I have left
public class Intake extends SubsystemBase {
  public final IntakeIO io;
  public final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean runIntake = true;

  public Intake(IntakeIO io) {
    this.io = io;
    setDefaultCommand(
        run(
            () -> io.set(0.0, 0.0)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Returns a command that intakes a note. */
  public Command intakeCommandSpeedMatch() {
    return runEnd(() -> {
      // get swerve velocity in x
      double speed = Units.metersToInches(Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond)); // "grr I HATE metric!" - every American ever
      // if the robot is moving, we'll want to match the speed of the robot,
      // so we first calculate the number of rotations per second we need to intake
      // luckily we already have the circumference from the Constants file
      double intakeRps = speed / Constants.IntakeConstants.CIRCUMFERENCE;
      // ...convert to RPM...
      double intakeRpm = intakeRps * 60;
      // ...and set the intake and cerealizer to that speed
      io.setVelocity(-intakeRpm, -intakeRpm);
    }, () -> {
      io.set(0.0, 0.0);
    });
  }

  /** Returns a command that intakes a note. Now with speed-agnostic output. */
  public Command intakeCommand() {
    return runEnd(() -> {
      io.setVelocity(Constants.IntakeConstants.INTAKE_SPEED, Constants.IntakeConstants.CEREALIZER_SPEED);
    }, () -> {
      io.set(0.0, 0.0);
    });
  }

  public void setVelocity(double intakeRpm, double cerealizerRpm) {
    io.setVelocity(intakeRpm, cerealizerRpm);
  }

  public void set(double intakePercent, double cerealizerPercent) {
    io.set(intakePercent, cerealizerPercent);
  }

  public void setIntakeVelocity(double intakeRpm) {
    io.setIntakeVelocity(intakeRpm);
  }

  public void setCerealizerVelocity(double cerealizerRpm) {
    io.setCerealizerVelocity(cerealizerRpm);
  }
  public void setCerealizer(double cerealizer) { io.setCerealizer(cerealizer);}
  public void setIntake(double a) { io.setIntake(a);}

  public void setRunIntake(boolean runIntake) {
    this.runIntake = runIntake;
  }


}