package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

/**
 *
 */
class RotateCommand(private val drivebase: SwerveSubsystem) : Command() {
    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivebase)
    }

    /**
     *
     */
    override fun initialize() {
    }

    /**
     *
     */
    override fun execute() {
        drivebase.drive(
            drivebase.swerveController.getTargetSpeeds(
                0.0, 0.0,
                1 * Math.PI,
                drivebase.heading.radians,
                drivebase.swerveDrive.maximumVelocity
            )
        )
    }

    /**
     *
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    /**
     *
     */
    override fun end(interrupted: Boolean) {
    }
}
