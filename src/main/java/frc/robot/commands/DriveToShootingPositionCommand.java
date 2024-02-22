package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveToShootingPositionCommand extends Command {
  private final SwerveSubsystem drivebase;

  public DriveToShootingPositionCommand(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivebase);
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(drivebase.driveToPose(
        new Pose2d(new Translation2d(2.720, 2.579), Rotation2d.fromDegrees(-175.312))));
  }

  @Override
  public void execute() {

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
