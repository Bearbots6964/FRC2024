package frc.robot.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import kotlin.math.sin

class EmoteCommand(private val swerveSubsystem: SwerveSubsystem) : Command() {

    var time: Double = 0.0
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
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
        // increase time by 0.02 seconds (20ms)
        time += 0.02
        // we want a sine function to oscillate between -45 and 45 every 0.75 seconds
        // we can use the sin function to do this
        val angle = sin(time * 2 * Math.PI / 0.75) * 1 / 2 * Math.PI
        // set the angle of the swerve drive to the angle we calculated
        swerveSubsystem.drive(Translation2d(), angle, false)
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
        return false
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
        // set the swerve drive to 0, 0, 0
        swerveSubsystem.driveCommand({ 0.0 }, { 0.0 }, { 0.0 })
    }
}
