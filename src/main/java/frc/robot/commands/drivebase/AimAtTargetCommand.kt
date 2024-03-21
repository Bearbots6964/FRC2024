package frc.robot.commands.drivebase

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.LimelightHelpers.getTX

/**
 *
 */
class AimAtTargetCommand(private val swerveSubsystem: SwerveSubsystem) : Command() {
    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem)
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
        swerveSubsystem.driveCommand(
            { 0.0 },
            { 0.0 },
            { -getTX("limelight-back") }) // Not sure if this will work, more math may be required.
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
