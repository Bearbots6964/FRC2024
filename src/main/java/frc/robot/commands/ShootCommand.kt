package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.Shooter
import java.util.function.DoubleSupplier

/**
 *
 */
class ShootCommand(private val shooter: Shooter, private val speed: DoubleSupplier) : Command() {
    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shooter)
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
        shooter.setVelocity(speed.asDouble, speed.asDouble)
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
        shooter[0.0] = 0.0
    }
}
