// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.util.Constants.AutonomousConstants
import frc.robot.util.LimelightHelpers
import org.littletonrobotics.junction.Logger
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.math.SwerveMath
import swervelib.parser.SwerveControllerConfiguration
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File
import java.util.function.DoubleSupplier
import kotlin.math.pow


/**
 *
 */
class SwerveSubsystem : SubsystemBase {
    private var lockRotation = false

    /**
     * Get the swerve drive object.
     *
     * @return [SwerveDrive] object.
     */
    /**
     * Swerve drive object.
     */
    @JvmField
    val swerveDrive: SwerveDrive

    /**
     * Maximum speed of the robot in meters per second, used to limit acceleration.
     */
    private var maximumSpeed: Double = Units.feetToMeters(120.5)

    /**
     * Initialize [SwerveDrive] with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    constructor(directory: File?) {


        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        val angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8)


        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
        //  The gear ratio is 6.75 motor revolutions per wheel rotation.
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        val driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4.0), 6.75)
        println("\"conversionFactor\": {")
        println("\t\"angle\": $angleConversionFactor,")
        println("\t\"drive\": $driveConversionFactor")
        println("}")


        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
        try {
            swerveDrive = SwerveParser(directory).createSwerveDrive(maximumSpeed)
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
        swerveDrive.setHeadingCorrection(false) // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation) // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        setupPathPlanner()


        swerveController.setMaximumAngularVelocity(Math.PI)

    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    constructor(driveCfg: SwerveDriveConfiguration?, controllerCfg: SwerveControllerConfiguration?) {
        swerveDrive = SwerveDrive(driveCfg, controllerCfg, maximumSpeed)
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    fun setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            { this.pose },  // Robot pose supplier
            { initialHolonomicPose: Pose2d? -> this.resetOdometry(initialHolonomicPose) },  // Method to reset odometry (will be called if your auto has a starting pose)
            { this.robotVelocity },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            { chassisSpeeds: ChassisSpeeds? -> this.setChassisSpeeds(chassisSpeeds) },  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                AutonomousConstants.TRANSLATION_PID,  // Translation PID constants
                AutonomousConstants.ANGLE_PID,  // Rotation PID constants
                4.5,  // Max module speed, in m/s
                swerveDrive.swerveDriveConfiguration.driveBaseRadiusMeters,  // Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
            ), {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                val alliance = DriverStation.getAlliance()
                if (alliance.isPresent) alliance.get() == Alliance.Red else false
            }, this // Reference to this subsystem to set requirements
        )
    }

    /**
     * Aim the robot at the target returned by the limelight.
     *
     * @param camera Name of the Limelight to communicate with.
     * @return A [Command] which will run the alignment.
     */
    fun aimAtTarget(camera: String?): Command {
        return run {
            if (LimelightHelpers.getTV(camera)) {
                drive(
                    getTargetSpeeds(
                        0.0, 0.0, Rotation2d.fromDegrees(LimelightHelpers.getTX(camera))
                    )
                ) // Not sure if this will work, more math may be required.
            }
        }
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return [AutoBuilder.followPath] path command.
     */
    fun getAutonomousCommand(pathName: String?): Command {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return PathPlannerAuto(pathName)
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target [Pose2d] to go to.
     * @return PathFinding command
     */
    fun driveToPose(pose: Pose2d?): Command {
        lockRotation = true
        val path = PathPlannerPath.fromPathFile("Go to red speaker")
        // Create the constraints to use while pathfinding
        val constraints = PathConstraints(
            swerveDrive.maximumVelocity, 4.0, swerveDrive.maximumAngularVelocity, Units.degreesToRadians(720.0)
        )

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            pose, constraints, 0.0,  // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    fun driveCommand(
        translationX: DoubleSupplier, translationY: DoubleSupplier, headingX: DoubleSupplier,
        headingY: DoubleSupplier,
    ): Command {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run {
            val xInput: Double = translationX.asDouble.pow(3.0) // Smooth controll out
            val yInput: Double = translationY.asDouble.pow(3.0) // Smooth controll out
            // Make the robot move
            driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    xInput,
                    yInput,
                    headingX.asDouble,
                    headingY.asDouble,
                    swerveDrive.odometryHeading.radians,
                    swerveDrive.maximumVelocity
                )
            )
        }
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */
    fun simDriveCommand(translationX: DoubleSupplier, translationY: DoubleSupplier, rotation: DoubleSupplier): Command {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run {
            // Make the robot move
            driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    translationX.asDouble,
                    translationY.asDouble,
                    rotation.asDouble * Math.PI,
                    swerveDrive.odometryHeading.radians,
                    swerveDrive.maximumVelocity
                )
            )
        }
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    fun sysIdDriveMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(), this, swerveDrive, 12.0
            ), 3.0, 5.0, 3.0
        )
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    fun sysIdAngleMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                SysIdRoutine.Config(), this, swerveDrive
            ), 3.0, 5.0, 3.0
        )
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    fun driveCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        angularRotationX: DoubleSupplier,
    ): Command {
        return run {
            // Make the robot move
            swerveDrive.drive(
                Translation2d(
                    translationX.asDouble.pow(3.0) * swerveDrive.maximumVelocity,
                    translationY.asDouble.pow(3.0) * swerveDrive.maximumVelocity
                ),
                angularRotationX.asDouble.pow(3.0) * swerveDrive.maximumAngularVelocity,
                true,
                false
            )
        }
    }
    fun botDriveCommand(
        translationX: DoubleSupplier,
        translationY: DoubleSupplier,
        angularRotationX: DoubleSupplier,
    ): Command {
        return run {
            // Make the robot move
            swerveDrive.drive(
                Translation2d(
                    translationX.asDouble.pow(3.0) * swerveDrive.maximumVelocity,
                    translationY.asDouble.pow(3.0) * swerveDrive.maximumVelocity
                ),
                angularRotationX.asDouble.pow(3.0) * swerveDrive.maximumAngularVelocity,
                false,
                false
            )
        }
    }

    /**
     * The primary method for controlling the drivebase.  Takes a [Translation2d] and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   [Translation2d] that is the commanded linear velocity of the robot, in meters per
     * second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     * torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     * (field North) and positive y is torwards the left wall when looking through the driver station
     * glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     * relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(
            translation, rotation, fieldRelative, false
        ) // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    private fun driveFieldOriented(velocity: ChassisSpeeds?) {
        swerveDrive.driveFieldOriented(velocity)
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented [ChassisSpeeds]
     */
    fun drive(velocity: ChassisSpeeds?) {
        swerveDrive.drive(velocity)
    }

    /**
     *
     */
    override fun periodic() {
        if (LimelightHelpers.getTV("limelight-back")) swerveDrive.addVisionMeasurement(
            LimelightHelpers.getBotPose2d(
                "limelight-back"
            ),
            Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-back") + LimelightHelpers.getLatency_Capture(
                "limelight-back"
            )) / 1000.0
        )
        Logger.recordOutput(
            "Limelight Pose",
            LimelightHelpers.getBotPose2d(
                "limelight-back"
            ),
        )
        Logger.recordOutput("Pose", pose)
        // PathPlannerLogging.logCurrentPose(getPose());

        swerveDrive.updateOdometry()

    }

    /**
     *
     */
    override fun simulationPeriodic() {
    }

    /**
     *
     */
    val kinematics: SwerveDriveKinematics
        /**
         * Get the swerve drive kinematics object.
         *
         * @return [SwerveDriveKinematics] of the swerve drive.
         */
        get() = swerveDrive.kinematics

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    fun resetOdometry(initialHolonomicPose: Pose2d?) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    /**
     *
     */
    val pose: Pose2d
        /**
         * Gets the current pose (position and rotation) of the robot, as reported by odometry.
         *
         * @return The robot's pose
         */
        get() = swerveDrive.pose

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds?) {
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    fun postTrajectory(trajectory: Trajectory?) {
        swerveDrive.postTrajectory(trajectory)
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    /**
     *
     */
    val heading: Rotation2d
        /**
         * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
         * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
         *
         * @return The yaw angle
         */
        get() = pose.rotation

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return [ChassisSpeeds] which can be sent to the Swerve Drive.
     */
    fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return swerveDrive.swerveController.getTargetSpeeds(
            xInput, yInput, headingX, headingY, heading.radians, maximumSpeed
        )
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a [Rotation2d].
     * @return [ChassisSpeeds] which can be sent to the Swerve Drive.
     */
    fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return swerveDrive.swerveController.getTargetSpeeds(
            xInput, yInput, angle.radians, heading.radians, maximumSpeed
        )
    }

    /**
     *
     */
    val fieldVelocity: ChassisSpeeds
        /**
         * Gets the current field-relative velocity (x, y and omega) of the robot
         *
         * @return A ChassisSpeeds object of the current field-relative velocity
         */
        get() = swerveDrive.fieldVelocity

    /**
     *
     */
    val robotVelocity: ChassisSpeeds
        /**
         * Gets the current velocity (x, y and omega) of the robot
         *
         * @return A [ChassisSpeeds] object of the current velocity
         */
        get() = swerveDrive.robotVelocity

    /**
     *
     */
    val swerveController: SwerveController
        /**
         * Get the [SwerveController] in the swerve drive.
         *
         * @return [SwerveController] from the [SwerveDrive].
         */
        get() = swerveDrive.swerveController

    /**
     *
     */
    val swerveDriveConfiguration: SwerveDriveConfiguration
        /**
         * Get the [SwerveDriveConfiguration] object.
         *
         * @return The [SwerveDriveConfiguration] fpr the current drive.
         */
        get() = swerveDrive.swerveDriveConfiguration

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    fun lock() {
        swerveDrive.lockPose()
    }

    /**
     *
     */
    val pitch: Rotation2d
        /**
         * Gets the current pitch angle of the robot, as reported by the imu.
         *
         * @return The heading as a [Rotation2d] angle
         */
        get() = swerveDrive.pitch


        /**
     * Add a fake vision reading for testing purposes.
         */
    fun addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(Pose2d(3.0, 3.0, Rotation2d.fromDegrees(65.0)), Timer.getFPGATimestamp())
    }

    companion object {
        fun maximumVelocity(): Double {
            return Units.feetToMeters(12.5)
        }
    }
}
