package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

  public IntakeCommand() {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    this.intakeSubsystem.suckIn();
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stop();
  }
}
