package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;


public class AimAndPickUpNoteCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final Intake intake;


  public AimAndPickUpNoteCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, Intake intake) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.intake = intake;

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem, this.visionSubsystem, this.intake);
  }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.set(0.5, 0.25);
      swerveSubsystem.getSwerveDrive().drive(new Translation2d(-visionSubsystem.limelight_range_proportional() / 5, 0), -visionSubsystem.limelight_aim_proportional_front() / 5, false, false);


  }

  @Override
  public boolean isFinished() {
    return !LimelightHelpers.getTV("limelight-front");
  }

  /**
   * The action to take when the command ends. Called when either the command
   * finishes normally -- that is, it is called when {@link #isFinished()} returns
   * true -- or when it is interrupted/canceled. This is where you may want to
   * wrap up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    intake.set(0,0);
    swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), 0, false, false);
  }
}
