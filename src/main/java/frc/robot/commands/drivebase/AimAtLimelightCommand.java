package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LimelightHelpers;


public class AimAtLimelightCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private boolean isRedAlliance;
  double rotateCW, speed;

  public AimAtLimelightCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;

    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem, this.visionSubsystem);
  }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-back", 1);
    isRedAlliance = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

    // Take the current angle and see if it's faster to rotate clockwise or counterclockwise to get to 180 degrees
    double currentAngle = swerveSubsystem.getHeading().getDegrees(); // (-180, 180]
    speed = 2.5;
    rotateCW = (currentAngle > 0) ? -speed : speed;

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled.
   * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {

    if (LimelightHelpers.getTV("limelight-back")) {
      swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), visionSubsystem.limelight_aim_proportional_back() / 5, false, false);
    } else {
      if (swerveSubsystem.getHeading().getDegrees() < 140 && swerveSubsystem.getHeading().getDegrees() > -140)
        swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), rotateCW, false, false);
      else swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), rotateCW / speed / 2, false, false);
    }

  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by
   * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always
   * return true will result in the command executing once and finishing immediately. It is
   * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
   * for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return (Math.abs(LimelightHelpers.getTX("limelight-back")) < 1.0) && LimelightHelpers.getTV("limelight-back");
  }

  /**
   * The action to take when the command ends. Called when either the command
   * finishes normally -- that is it is called when {@link #isFinished()} returns
   * true -- or when  it is interrupted/canceled. This is where you may want to
   * wrap up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex("limelight-back", 0);
    swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), 0, false, false);
  }
}
