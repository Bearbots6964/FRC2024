package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Constants;

import java.util.function.DoubleSupplier;
import java.util.prefs.PreferencesFactory;


public class IntakeWithSwerveCommand extends Command {
   private final Intake intake;

   public IntakeWithSwerveCommand(Intake intake) {
      this.intake = intake;
      // each subsystem used by the command must be passed into the
      // addRequirements() method (which takes a vararg of Subsystem)
      addRequirements(this.intake);
   }

   @Override
   public void initialize() {
      // get swerve velocity in x
      double speedSwerve = Units.metersToInches(Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond)); // "grr I HATE metric!" - every American ever
      // if the robot is moving, we'll want to match the speed of the robot,
      // so we first calculate the number of rotations per second we need to intake
      // luckily we already have the circumference from the Constants file
      double intakeRps = speedSwerve / Constants.IntakeConstants.CIRCUMFERENCE;
      // ...convert to RPM...
      double intakeRpm = intakeRps * 60;
      // ...and set the intake and cerealizer to that speed
      intake.setVelocity(intakeRpm, intakeRpm);
   }

   @Override
   public void execute() {
      // get swerve velocity in x
      double speedSwerve = Units.metersToInches(Math.abs(SwerveSubsystem.getInstance().getRobotVelocity().vxMetersPerSecond)); // "grr I HATE metric!" - every American ever
      // if the robot is moving, we'll want to match the speed of the robot,
      // so we first calculate the number of rotations per second we need to intake
      // luckily we already have the circumference from the Constants file
      double intakeRps = speedSwerve / Constants.IntakeConstants.CIRCUMFERENCE;
      // ...convert to RPM...
      double intakeRpm = intakeRps * 60;
      // ...and set the intake and cerealizer to that speed
      intake.setVelocity(intakeRpm, intakeRpm);
   }

   @Override
   public boolean isFinished() {
      // TODO: Make this return true when this Command no longer needs to run execute()
      return false;
   }

   @Override
   public void end(boolean interrupted) {
      intake.set(0.0, 0.0);
   }
}
