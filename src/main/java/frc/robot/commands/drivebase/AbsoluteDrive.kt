// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivebase

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.Constants
import swervelib.SwerveController
import swervelib.math.SwerveMath
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class AbsoluteDrive(
    private val swerve: SwerveSubsystem,
    private val vX: DoubleSupplier,
    private val vY: DoubleSupplier,
    private val headingHorizontal: DoubleSupplier,
    private val headingVertical: DoubleSupplier
) : Command() {
    private var initRotation = false

    /**
     * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve            The swerve drivebase subsystem.
     * @param vX                DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1
     * to 1 with deadband already accounted for.  Positive X is away from the alliance wall.
     * @param vY                DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1
     * to 1 with deadband already accounted for.  Positive Y is towards the left wall when
     * looking through the driver station glass.
     * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle. In the
     * robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with
     * no deadband.  Positive is towards the left wall when looking through the driver station
     * glass.
     * @param headingVertical   DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
     * robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1
     * with no deadband. Positive is away from the alliance wall.
     */
    init {
        addRequirements(swerve)
    }

    /**
     *
     */
    override fun initialize() {
        initRotation = true
    }

    /**
     *
     */// Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        // Get the desired chassis speeds based on a 2 joystick module.

        var desiredSpeeds = swerve.getTargetSpeeds(
            vX.asDouble, vY.asDouble,
            headingHorizontal.asDouble,
            headingVertical.asDouble
        )

        // Prevent Movement After Auto
        if (initRotation) {
            if (headingHorizontal.asDouble == 0.0 && headingVertical.asDouble == 0.0) {
                // Get the curretHeading
                val firstLoopHeading = swerve.heading

                // Set the Current Heading to the desired Heading
                desiredSpeeds = swerve.getTargetSpeeds(0.0, 0.0, firstLoopHeading.sin, firstLoopHeading.cos)
            }
            //Dont Init Rotation Again
            initRotation = false
        }

        // Limit velocity to prevent tippy
        var translation = SwerveController.getTranslation2d(desiredSpeeds)
        translation = SwerveMath.limitVelocity(
            translation, swerve.fieldVelocity, swerve.pose,
            Constants.LOOP_TIME, Constants.ROBOT_MASS, listOf(Constants.CHASSIS),
            swerve.swerveDriveConfiguration
        )
        SmartDashboard.putNumber("LimitedTranslation", translation.x)
        SmartDashboard.putString("Translation", translation.toString())

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true)
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
