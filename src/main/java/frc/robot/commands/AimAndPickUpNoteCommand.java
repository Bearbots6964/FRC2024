package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;


public class AimAndPickUpNoteCommand extends Command {
  private final Intake intake;
  private final SwerveSubsystem swerveSubsystem;
  private final String limelightName = "limelight-front";

  public AimAndPickUpNoteCommand(Intake intake, SwerveSubsystem swerveSubsystem) {
    this.intake = intake;
    this.swerveSubsystem = swerveSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intake, this.swerveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(LimelightHelpers.getTV(limelightName))
      swerveSubsystem.drive(new Translation2d(VisionSubsystem.getInstance().limelight_range_proportional(), 0), VisionSubsystem.getInstance().limelight_aim_proportional_front(), false);
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
    intake.set(0,0);
  }
}
