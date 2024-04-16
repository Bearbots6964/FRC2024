package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.subsystems.intake.Intake
import org.littletonrobotics.junction.Logger

/**
 *
 */
class IntakeCommand(private val intake: Intake) : Command() {
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
        intake.set(0.55, 0.375, -0.375)
        Logger.recordOutput("Proximity", (intake.colorSensorProximity ?: return).asDouble)
        if ((intake.colorSensorProximity ?: return).asDouble <= 150) RobotContainer.hasNote = false
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
        if ((intake.colorSensorProximity ?: return).asDouble > 150) RobotContainer.hasNote = true
        RobotContainer.rumbleShooterControllerTwiceNotifier.startSingle(0.0)

    }
}
