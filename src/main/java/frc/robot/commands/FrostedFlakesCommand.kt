package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIOSparkMax

class FrostedFlakesCommand(private val intake: Intake, private val keepGoing: Boolean) : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(intake)
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    override fun initialize() {}

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until [isFinished] returns true.)
     */
    override fun execute() {
        intake.set(0.0, -1.0)
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its [end] method.
     *
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use [InstantCommand][edu.wpi.first.wpilibj2.command.InstantCommand]
     * for such an operation.
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return IntakeIOSparkMax.colorSensor.proximity <= 150 && !keepGoing
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is, it is called when [isFinished] returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {
        intake.set(0.0, 0.0)
    }
}
