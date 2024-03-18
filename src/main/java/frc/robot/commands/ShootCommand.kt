package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;


public class ShootCommand extends Command {
  private final Shooter shooter;
  private final DoubleSupplier speed;


  public ShootCommand(Shooter shooter, DoubleSupplier speed) {
    this.shooter = shooter;
    this.speed = speed;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    shooter.setVelocity(speed.getAsDouble(), speed.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0, 0.0);
  }
}
