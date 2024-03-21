package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimberSubsystem

/**
 *
 */
class ClimbCommand : Command() {
    private val climberSubsystem: ClimberSubsystem = ClimberSubsystem.instance

    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem)
    }

    /**
     *
     */
    override fun initialize() {}

    /**
     *
     */
    override fun execute() {}

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
    override fun end(interrupted: Boolean) {}
}
