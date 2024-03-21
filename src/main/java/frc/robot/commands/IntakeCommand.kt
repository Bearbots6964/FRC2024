package frc.robot.commands

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.subsystems.intake.Intake
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

/**
 *
 */
class IntakeCommand(private val intake: Intake, private val speed: DoubleSupplier) : Command() {
    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake)
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
        intake.set(0.25, -0.25)
        Logger.recordOutput("Proximity", (intake.colorSensorProximity ?: return).asDouble)
    }

    /**
     *
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return intake.colorSensorProximity!!.asDouble > 150
    }

    /**
     *
     */
    override fun end(interrupted: Boolean) {
        intake.set(0.0, 0.0)

        RobotContainer.rumbleShooterControllerTwiceNotifier.startSingle(0.0)

    }
}
