package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  public ShootCommand() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.shooterSubsystem, this.visionSubsystem);
  }

  @Override
  public void initialize() {
    this.shooterSubsystem.shoot();
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
  }
}
