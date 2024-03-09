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

  @Override
  public boolean isFinished() {
    return (Math.abs(LimelightHelpers.getTX("limelight-back")) < 1.0) && LimelightHelpers.getTV("limelight-back");
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex("limelight-back", 0);
    swerveSubsystem.getSwerveDrive().drive(new Translation2d(0, 0), 0, false, false);
  }
}
