package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import java.util.function.DoubleSupplier

/**
 *
 */
class ReverseIntakeCommand(private val intake: Intake, private val shooter: Shooter, private val speed: DoubleSupplier) : Command() {
    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.shooter)
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
        intake[-0.25] = 0.25
        shooter.set(-0.1, -0.1)
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
        intake[0.0] = 0.0
        shooter.set(0.0, 0.0)
    }
}
