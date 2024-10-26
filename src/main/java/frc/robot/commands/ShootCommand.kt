package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.Shooter
import java.util.function.DoubleSupplier
import kotlin.math.pow

/**
 *
 */
class ShootCommand(
    private val shooter: Shooter,
    private val speed: DoubleSupplier,
    private val shooterIsRunning: () -> Unit,
    private val shooterIsNotRunning: () -> Unit,
    private val leftTrigger: DoubleSupplier,
) : Command() {

    val maxRpm = 3500.0

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
        if (leftTrigger.asDouble >= 0.95) {
            shooter.setVelocity(0.75 * maxRpm, 0.75 * maxRpm)
        } else
            shooter.setVelocity(speed.asDouble.pow(2.00) * maxRpm, speed.asDouble.pow(2.00) * maxRpm)
        if (speed.asDouble > 0.05) {
            shooterIsRunning()
        } else {
            shooterIsNotRunning()
        }
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
