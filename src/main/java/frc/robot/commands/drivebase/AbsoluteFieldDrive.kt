// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivebase

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.Constants
import swervelib.SwerveController
import swervelib.math.SwerveMath
import java.util.List
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class AbsoluteFieldDrive(
    private val swerve: SwerveSubsystem, private val vX: DoubleSupplier, private val vY: DoubleSupplier,
    private val heading: DoubleSupplier
) : Command() {
    /**
     * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
     * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
     * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
     * will rotate to.
     *
     * @param swerve  The swerve drivebase subsystem.
     * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
     * deadband already accounted for.  Positive X is away from the alliance wall.
     * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
     * deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
     * station glass.
     * @param heading DoubleSupplier that supplies the robot's heading angle.
     */
    init {
        addRequirements(swerve)
    }

    /**
     *
     */
    override fun initialize() {
    }

    /**
     *
     */// Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        // Get the desired chassis speeds based on a 2 joystick module.

        val desiredSpeeds = swerve.getTargetSpeeds(
            vX.asDouble, vY.asDouble,
            Rotation2d(heading.asDouble * Math.PI)
        )

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
