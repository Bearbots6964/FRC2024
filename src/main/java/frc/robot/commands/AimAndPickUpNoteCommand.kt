package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.VisionSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.LimelightHelpers

/**
 *
 */
class AimAndPickUpNoteCommand(
    private val swerveSubsystem: SwerveSubsystem,
    private val visionSubsystem: VisionSubsystem,
    private val intake: Intake,
) : Command() {
    val aimingPid: PIDController = PIDController(0.1, 0.0, 0.0)

    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem, this.visionSubsystem, this.intake)
    }


    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    override fun initialize() {
    }

    /**
     *
     */
    override fun execute() {
        intake.set(0.25, -0.25)
        if (LimelightHelpers.getTV("limelight-front"))
            swerveSubsystem.swerveDrive.drive(
                Translation2d(-visionSubsystem.limelightRangeProportional() / 5, 0.0),
                aimingPid.calculate(LimelightHelpers.getTX("limelight-front"), 0.0) / 5,
                false,
                false
            )

    }


/**
 *
 */
override fun isFinished(): Boolean {
    return !LimelightHelpers.getTV("limelight-front")
}

/**
 * The action to take when the command ends. Called when either the command
 * finishes normally -- that is, it is called when [.isFinished] returns
 * true -- or when it is interrupted/canceled. This is where you may want to
 * wrap up loose ends, like shutting off a motor that was being used in the command.
 *
 * @param interrupted whether the command was interrupted/canceled
 */
override fun end(interrupted: Boolean) {
    intake[0.0] = 0.0
    swerveSubsystem.swerveDrive.drive(Translation2d(0.0, 0.0), 0.0, false, false)
}
}
