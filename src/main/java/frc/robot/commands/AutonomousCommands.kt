package frc.robot.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.util.Constants

// this all feels like janky garbage but I think it's gonna score at least 2 points

public class AutonomousCommands(
        swerveSubsystem: SwerveSubsystem,
        armSubsystem: ArmSubsystem,
        shooter: Shooter,
        intake: Intake,
) : SequentialCommandGroup() {
    init {
        addCommands(
                swerveSubsystem.getAutonomousCommand("start-to-speaker"),
                InstantCommand({armSubsystem.moveArmToAngle(50.0)}, armSubsystem),
                ShootCommand(shooter, {Constants.ShooterConstants.MAX_VELOCITY}, {}, {}),
                swerveSubsystem.getAutonomousCommand("speaker-to-note-2"),
                // value doesn't matter, `speed` is unused
                IntakeCommand(intake),
        )
    }
}
