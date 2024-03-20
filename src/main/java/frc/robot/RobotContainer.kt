// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.*
import frc.robot.commands.drivebase.AbsoluteDriveAdv
import frc.robot.commands.drivebase.AbsoluteFieldDrive
import frc.robot.commands.drivebase.AimAtLimelightCommand
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.VisionSubsystem
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIOSparkMax
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIOSparkFlex
import frc.robot.util.Constants.OperatorConstants
import java.io.File

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
     val drivebase = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "c"))
    private val intake = Intake(IntakeIOSparkMax())
    private val shooter = Shooter(ShooterIOSparkFlex())
    private val armSubsystem: ArmSubsystem = ArmSubsystem.instance
    private val intakeCommand: Command
    private val shootCommand: Command
    private val aimAtLimelightCommand: Command
    private val rotateCommand: Command
    private val driveToPoseCommand: Command
    private val aimAndPickUpNoteCommand: Command
    private val armSysIdCommand: Command
    private val moveArmCommand: Command
    private val reverseIntakeCommand: Command
    private val homeArmCommand: Command
    private val frostedFlakesCommand: Command
    private val moveArmToAmpCommand: Command
    private val overrideShootCommand: Command
    val driveBotOrientedAngularVelocity: Command

    private val absoluteFieldDrive: Command

    private var shooterRunning: Boolean = false
    var hasNote: Boolean = false





    /**
     *
     */


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        intakeCommand = IntakeCommand(intake) { MathUtil.applyDeadband(shooterXbox.leftTriggerAxis, 0.1) }
        shootCommand = ShootCommand(shooter, {MathUtil.applyDeadband(shooterXbox.rightTriggerAxis, 0.1)},
            { shooterRunning = true }, { shooterRunning = false })
        aimAtLimelightCommand = AimAtLimelightCommand(drivebase, VisionSubsystem.instance)
        rotateCommand = RotateCommand(drivebase)
        driveToPoseCommand = ShootCommand(shooter, { 4000.0 },
            { shooterRunning = true }, { shooterRunning = false }).alongWith(
            drivebase.driveToPose(
                Pose2d(
                    Translation2d(
                        2.720, 2.579
                    ), Rotation2d.fromDegrees(180.0)
                )
            ).andThen(drivebase.driveToPose(Pose2d(Translation2d(2.720, 2.579), Rotation2d.fromDegrees(180.0))))
                .andThen(AimAtLimelightCommand(drivebase, VisionSubsystem.instance))
                .andThen(IntakeCommand(intake, { 1.0 }))
        )
        aimAndPickUpNoteCommand = AimAndPickUpNoteCommand(drivebase, VisionSubsystem.instance, intake)
        armSysIdCommand = ArmSubsystem.generateSysIdCommand(armSubsystem.sysId, 2.0, 3.5, 1.5)
        moveArmCommand = MoveArmCommand { MathUtil.applyDeadband(shooterXbox.rightY, 0.1) }
        reverseIntakeCommand = ReverseIntakeCommand(intake) { MathUtil.applyDeadband(shooterXbox.leftTriggerAxis, 0.1) }
        homeArmCommand = HomeArmCommand()
        frostedFlakesCommand = FrostedFlakesCommand(intake, false)
        overrideShootCommand = FrostedFlakesCommand(intake, true)

        moveArmToAmpCommand = MoveArmToAmpCommand(armSubsystem)
        val invert = invert

        absoluteFieldDrive = AbsoluteFieldDrive(drivebase,
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * invert })
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        val driveFieldOrientedDirectAngle = drivebase.driveCommand({
            MathUtil.applyDeadband(
                driverXbox.leftY,
                OperatorConstants.LEFT_Y_DEADBAND
            ) * invert
        },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * invert },
            { driverXbox.rightY * invert })
        val closedAbsoluteDriveAdv = AbsoluteDriveAdv(drivebase,
            { MathUtil.applyDeadband(driverXbox.leftY * invert, OperatorConstants.LEFT_Y_DEADBAND) },
            { MathUtil.applyDeadband(driverXbox.leftX * invert, OperatorConstants.LEFT_X_DEADBAND) },
            { MathUtil.applyDeadband(driverXbox.rightX * invert, OperatorConstants.RIGHT_X_DEADBAND) },
            { driverXbox.yButtonPressed },
            { driverXbox.aButtonPressed },
            { driverXbox.xButtonPressed },
            { driverXbox.bButtonPressed })

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        val driveFieldOrientedAnglularVelocity = drivebase.driveCommand({
            -MathUtil.applyDeadband(
                driverXbox.leftY,
                OperatorConstants.LEFT_Y_DEADBAND
            ) * invert
        },
            { -MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * 2 * invert })


        driveBotOrientedAngularVelocity = drivebase.botDriveCommand({
            MathUtil.applyDeadband(
                driverXbox.leftY,
                OperatorConstants.LEFT_Y_DEADBAND
            ) * invert
        },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * 2.0 * invert })

        val driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand({
            MathUtil.applyDeadband(
                driverXbox.leftY,
                OperatorConstants.LEFT_Y_DEADBAND
            ) * invert
        },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.getRawAxis(2) * invert })

        val sysIdForDrive = drivebase.sysIdDriveMotorCommand()
        val sysIdForAngle = drivebase.sysIdAngleMotorCommand()

        val driveFieldOrientedDirectAngleForTesting =
            drivebase.driveCommand({ MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) },
                { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) },
                { driverXbox.rightX },
                { driverXbox.rightY })

        drivebase.defaultCommand =
            if (!RobotBase.isSimulation()) driveFieldOrientedAnglularVelocity else driveFieldOrientedDirectAngleSim

        // Configure the trigger bindings
        configureBindings()
    }

    private val invert: Int
        get() {
            val alliance = DriverStation.getAlliance()
            val invert = if (alliance.isPresent && alliance.get() == Alliance.Red) {
                -1
            } else {
                1
            }


            return invert
        }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary predicate, or via the
     * named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        JoystickButton(driverXbox, Button.kA.value).onTrue((InstantCommand({ drivebase.zeroGyro() })))
        JoystickButton(driverXbox, Button.kX.value).whileTrue(aimAtLimelightCommand)
        JoystickButton(driverXbox, Button.kB.value).whileTrue(driveToPoseCommand)
        JoystickButton(driverXbox, Button.kY.value).whileTrue(rotateCommand)
        JoystickButton(driverXbox, Button.kLeftBumper.value).whileTrue(driveBotOrientedAngularVelocity)
        JoystickButton(driverXbox, Button.kY.value).whileTrue(aimAndPickUpNoteCommand)


        // for the intake command, if the shooter is running, execute the frosted flakes command, and otherwise, execute the intake command
        JoystickButton(shooterXbox, Button.kLeftBumper.value).whileTrue(Commands.either(frostedFlakesCommand, intakeCommand) { shooterRunning })
        JoystickButton(shooterXbox, Button.kRightBumper.value).whileTrue(reverseIntakeCommand)
        JoystickButton(shooterXbox, Button.kA.value).onTrue(homeArmCommand)
        JoystickButton(shooterXbox, Button.kB.value).whileTrue(overrideShootCommand)
        JoystickButton(shooterXbox, Button.kY.value).onTrue(moveArmToAmpCommand)
        JoystickButton(shooterXbox, Button.kX.value).onTrue( Commands.runOnce({ CommandScheduler.getInstance().cancel(homeArmCommand, moveArmToAmpCommand, moveArmCommand) }))



        shooter.defaultCommand = shootCommand
        armSubsystem.defaultCommand = moveArmCommand
    }

    /**
     *
     */
    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() =// An example command will be run in autonomous
            drivebase.getAutonomousCommand("New Auto")

    /**
     *
     */
    fun setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    /**
     *
     */
    fun setMotorBrake(brake: Boolean) {
        drivebase.setMotorBrake(brake)
    }

    companion object {
        var driverXbox: XboxController = XboxController(0)

        /**
         *
         */
        var shooterXbox: XboxController = XboxController(1)


        val rumbleShooterControllerTwiceNotifier = Notifier {
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.5)
            Thread.sleep(125)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.0)
            // wait 0.25 seconds
            Thread.sleep(250)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.5)
            Thread.sleep(125)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.0)

        }
    }

}