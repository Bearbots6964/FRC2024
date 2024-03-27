// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivebase

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.Constants
import org.littletonrobotics.junction.Logger
import swervelib.SwerveController
import swervelib.math.SwerveMath
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.abs

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction to face
 */
class AbsoluteDriveAdv(
    private val swerve: SwerveSubsystem,
    private val vX: DoubleSupplier,
    private val vY: DoubleSupplier,
    private val headingAdjust: DoubleSupplier,
    private val lookAway: BooleanSupplier,
    private val lookTowards: BooleanSupplier,
    private val lookLeft: BooleanSupplier,
    private val lookRight: BooleanSupplier
) : Command() {
    private var resetHeading = false

    /**
     * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. Heading Adjust changes the current heading after being
     * multipied by a constant. The look booleans are shortcuts to get the robot to face a certian direction. Based off of
     * ideas in https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
     *
     * @param swerve        The swerve drivebase subsystem.
     * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
     * with deadband already accounted for.  Positive X is away from the alliance wall.
     * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
     * with deadband already accounted for.  Positive Y is towards the left wall when looking through
     * the driver station glass.
     * @param headingAdjust DoubleSupplier that supplies the component of the robot's heading angle that should be
     * adjusted. Should range from -1 to 1 with deadband already accounted for.
     * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
     * facing
     * @param lookTowards   Face the robot towards the driver
     * @param lookLeft      Face the robot left
     * @param lookRight     Face the robot right
     */
    init {
        addRequirements(swerve)
    }

    /**
     *
     */
    override fun initialize() {
        resetHeading = true
    }

    /**
     *
     */// Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        var headingX = 0.0
        var headingY = 0.0

        // These are written to allow combinations for 45 angles
        // Face Away from Drivers
        if (lookAway.asBoolean) {
            headingY = -1.0
        }
        // Face Right
        if (lookRight.asBoolean) {
            headingX = 1.0
        }
        // Face Left
        if (lookLeft.asBoolean) {
            headingX = -1.0
        }
        // Face Towards the Drivers
        if (lookTowards.asBoolean) {
            headingY = 1.0
        }

        // Prevent Movement After Auto
        if (resetHeading) {
            if (headingX == 0.0 && headingY == 0.0 && abs(headingAdjust.asDouble) > 0) {
                // Get the curret Heading
                val currentHeading = swerve.heading

                // Set the Current Heading to the desired Heading
                headingX = currentHeading.sin
                headingY = currentHeading.cos
            }
            //Dont reset Heading Again
            resetHeading = false
        }

        val desiredSpeeds = swerve.getTargetSpeeds(vX.asDouble, vY.asDouble, headingX, headingY)

        // Limit velocity to prevent tippy
        var translation = SwerveController.getTranslation2d(desiredSpeeds)
        translation = SwerveMath.limitVelocity(
            translation, swerve.fieldVelocity, swerve.pose,
            Constants.LOOP_TIME, Constants.ROBOT_MASS, listOf(Constants.CHASSIS),
            swerve.swerveDriveConfiguration
        )
        Logger.recordOutput("AbsoluteDriveAdv/LimitedTranslation", translation.x)
        Logger.recordOutput("AbsoluteDriveAdv/Translation", translation.toString())

        // Make the robot move
        if (headingX == 0.0 && headingY == 0.0 && abs(headingAdjust.asDouble) > 0) {
            resetHeading = true
            swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.asDouble), true)
        } else {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true)
        }
    }

    /**
     *
     */// Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }

    /**
     *
     */// Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
