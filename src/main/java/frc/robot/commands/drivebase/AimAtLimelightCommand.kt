package frc.robot.commands.drivebase

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.VisionSubsystem
import frc.robot.LimelightHelpers.getTV
import frc.robot.LimelightHelpers.getTX
import frc.robot.LimelightHelpers.setPipelineIndex
import kotlin.math.abs

/**
 *
 */
class AimAtLimelightCommand(
    private val swerveSubsystem: SwerveSubsystem,
    private val visionSubsystem: VisionSubsystem
) : Command() {
    private var isRedAlliance = false
    /**
     *
     */
    private var rotateCW: Double = 0.0
    /**
     *
     */
    var speed: Double = 1.0

    init {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem, this.visionSubsystem)
    }


    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    override fun initialize() {
        setPipelineIndex("limelight-back", 1)

        // Take the current angle and see if it's faster to rotate clockwise or counterclockwise to get to 180 degrees
        val currentAngle = swerveSubsystem.heading.degrees // (-180, 180]
        speed = 2.5
        rotateCW = if ((currentAngle > 0)) -speed else speed
    }

    /**
     *
     */
    override fun execute() {
        if (getTV("limelight-back")) {
            swerveSubsystem.swerveDrive.drive(
                Translation2d(0.0, 0.0),
                -visionSubsystem.limelightAimProportionalBack() * 3,
                false,
                false
            )
        } else {
            if (swerveSubsystem.heading.degrees < 140 && swerveSubsystem.heading.degrees > -140) swerveSubsystem.swerveDrive.drive(
                Translation2d(0.0, 0.0),
                rotateCW * 3,
                false,
                false
            )

            else swerveSubsystem.swerveDrive.drive(Translation2d(0.0, 0.0), -rotateCW * 3, false, false)
        }
    }

    /**
     *
     */
    override fun isFinished(): Boolean {
        return (abs(getTX("limelight-back")) < 3.0) && getTV("limelight-back")
    }

    /**
     *
     */
    override fun end(interrupted: Boolean) {
        setPipelineIndex("limelight-back", 0)
        swerveSubsystem.swerveDrive.drive(Translation2d(0.0, 0.0), 0.0, false, false)
    }
}
