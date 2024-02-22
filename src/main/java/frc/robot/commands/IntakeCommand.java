package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Constants;

import java.util.function.DoubleSupplier;
import java.util.prefs.PreferencesFactory;


public class IntakeCommand extends Command {
  private final Intake intake;
  private final DoubleSupplier speed;

  public IntakeCommand(Intake intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.setVelocity(1 /*revolutions per minute*/ * 15 /* revolutions per turn */ * 60 /* seconds per minute */ * 2 /* turns per second*/,
        1.25  /*revolutions per minute*/ * 20 /* revolutions per turn */ * 60 /* seconds per minute */ * 2 /* turns per second*/);

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0.0, 0.0);
  }
}
