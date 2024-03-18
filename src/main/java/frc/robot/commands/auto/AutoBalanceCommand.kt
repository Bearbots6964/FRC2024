package frc.robot.commands.auto

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem

/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * [...](https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java)
 */
class AutoBalanceCommand(private val swerveSubsystem: SwerveSubsystem) : Command() {
    private val controller = PIDController(1.0, 0.0, 0.0)

    init {
        controller.setTolerance(1.0)
        controller.setpoint = 0.0
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem)
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    override fun initialize() {
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
     * until [.isFinished]) returns true.)
     */
    override fun execute() {
        SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint())

        val translationVal = MathUtil.clamp(
            controller.calculate(swerveSubsystem.pitch.degrees, 0.0), -0.5,
            0.5
        )
        swerveSubsystem.drive(Translation2d(translationVal, 0.0), 0.0, true)
    }

    /**
     *
     *
     * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
     * the scheduler will call its [.end] method.
     *
     *
     * Returning false will result in the command never ending automatically. It may still be cancelled manually or
     * interrupted by another command. Hard coding this command to always return true will result in the command executing
     * once and finishing immediately. It is recommended to use *
     * [InstantCommand][edu.wpi.first.wpilibj2.command.InstantCommand] for such an operation.
     *
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        return controller.atSetpoint()
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
     * when [.isFinished] returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
     * up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {
        swerveSubsystem.lock()
    }
}
