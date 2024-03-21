package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class RotateCommand extends Command {
  private final SwerveSubsystem drivebase;

  public RotateCommand(SwerveSubsystem swerveSubsystem) {
    this.drivebase = swerveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivebase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    drivebase.drive( drivebase.getSwerveController().getTargetSpeeds(0, 0,
        1 * Math.PI,
        drivebase.getHeading().getRadians(),
        drivebase.getSwerveDrive().getMaximumVelocity()));
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
