package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;


public class AimAtTargetCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;

  public AimAtTargetCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    swerveSubsystem.driveCommand(() -> 0.0, () -> 0.0, () -> Math.cos(Units.degreesToRadians(LimelightHelpers.getTX("limelight-back"))), () -> Math.sin(Units.degreesToRadians(LimelightHelpers.getTX("limelight-back")))); // Not sure if this will work, more math may be required.
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
