// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
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
import frc.robot.util.UtilityCommand
import java.io.File

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    val drivebase: SwerveSubsystem = SwerveSubsystem(File(Filesystem.getDeployDirectory(), "g"))

    //<editor-fold desc="Command declaration">
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
    private val autoShootCommand: Command
    private val pushNoteStatus: Command
    private val emoteCommand: Command
    private val absoluteFieldDrive: Command
    private val restartRioCommand: Command
    //</editor-fold>

    private var shooterRunning: Boolean = false


    private var autoSelected: String = String()
    var chooser: SendableChooser<Command> = SendableChooser()


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {

        //<editor-fold desc="PathPlanner auto command registration">
        NamedCommands.registerCommands(
            mapOf(
                Pair("intakeCommand", IntakeCommand(intake)),
                Pair(
                    "shootCommand",
                    ShootCommand(shooter, { 0.9 }, { shooterRunning = true }, { shooterRunning = false }, { 0.0 })
                ),
                Pair(
                    "shootCommand2",
                    ShootCommand(shooter, { 0.9 }, { shooterRunning = true }, { shooterRunning = false }, { 0.0 })
                ),
                Pair(
                    "shootHarderCommand",
                    ShootCommand(shooter, { 0.95 }, { shooterRunning = true }, { shooterRunning = false }, { 0.0 })
                ),
                Pair("moveArmCommand", InstantCommand({ armSubsystem.moveArmToAngle(62.0) }, armSubsystem)),
                Pair("moveArmCommand2", InstantCommand({ armSubsystem.moveArmToAngle(52.0) }, armSubsystem)),
                Pair("frostedFlakesCommand", FrostedFlakesCommand(intake, false)),
                Pair("homeArmCommand", InstantCommand({ armSubsystem.moveArmToAngle(40.0) }, armSubsystem)),
                Pair("homeArmCommandButABitUp", InstantCommand({ armSubsystem.moveArmToAngle(43.5) }, armSubsystem)),

                Pair(
                    "moveToCenter",
                    drivebase.driveToPose(Pose2d(Translation2d(8.33, 0.8), Rotation2d(Math.PI)))
                        .alongWith(IntakeCommand(intake))
                ),
                Pair("aim", AimAtLimelightCommand(drivebase, VisionSubsystem.instance)),
                Pair("shootABitSlowerCommand", ShootCommand(shooter, { 0.0 }, { shooterRunning = true }, { shooterRunning = false }, { 1.0 })),
                        Pair("homeArmCommandButABitLessUp", InstantCommand({ armSubsystem.moveArmToAngle(41.75) }, armSubsystem)),
            )
        )
        //</editor-fold>
        //<editor-fold desc="Command instantiation">
        intakeCommand = IntakeCommand(intake)
        shootCommand = ShootCommand(shooter, { MathUtil.applyDeadband(shooterXbox.rightTriggerAxis, 0.1) },
            { shooterRunning = true }, { shooterRunning = false }, { shooterXbox.leftTriggerAxis })
        aimAtLimelightCommand = AimAtLimelightCommand(drivebase, VisionSubsystem.instance)
        rotateCommand = RotateCommand(drivebase)
        driveToPoseCommand = ShootCommand(shooter, { 4000.0 },
            { shooterRunning = true }, { shooterRunning = false }, { 0.0 }).alongWith(
            drivebase.driveToPose(
                Pose2d(
                    Translation2d(
                        2.720, 2.579
                    ), Rotation2d.fromDegrees(180.0)
                )
            ).andThen(drivebase.driveToPose(Pose2d(Translation2d(2.720, 2.579), Rotation2d.fromDegrees(180.0))))
                .andThen(AimAtLimelightCommand(drivebase, VisionSubsystem.instance))
                .andThen(IntakeCommand(intake))
        )
        aimAndPickUpNoteCommand = AimAndPickUpNoteCommand(drivebase, VisionSubsystem.instance, intake)
        armSysIdCommand = ArmSubsystem.generateSysIdCommand(armSubsystem.sysId, 2.0, 3.5, 1.5)
        moveArmCommand = MoveArmCommand { MathUtil.applyDeadband(shooterXbox.rightY, 0.1) }
        reverseIntakeCommand =
            ReverseIntakeCommand(intake, shooter) { MathUtil.applyDeadband(shooterXbox.leftTriggerAxis, 0.1) }
        homeArmCommand = HomeArmCommand(armSubsystem)
        frostedFlakesCommand = FrostedFlakesCommand(intake, false)
        overrideShootCommand = FrostedFlakesCommand(intake, true)
        autoShootCommand =
            ShootCommand(
                shooter,
                { return@ShootCommand 0.1 },
                { shooterRunning = true },
                { shooterRunning = false },
                { 0.0 })
        moveArmToAmpCommand = MoveArmToAmpCommand(armSubsystem)
        emoteCommand = EmoteCommand(drivebase)
        restartRioCommand = RestartRioCommand()

        //</editor-fold>
        //<editor-fold desc="Swerve commands and setup">
        absoluteFieldDrive = AbsoluteFieldDrive(drivebase,
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * invert })

        pushNoteStatus = UtilityCommand { SmartDashboard.putBoolean("Note", hasNote) }
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
            { -driverXbox.rightX })


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


        val driveFieldOrientedDirectAngleForTesting =
            drivebase.driveCommand({ MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) },
                { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) },
                { driverXbox.rightX },
                { driverXbox.rightY })

        drivebase.defaultCommand =
            driveFieldOrientedAnglularVelocity
        //</editor-fold>

        // Configure the trigger bindings
        configureBindings()

        chooser = AutoBuilder.buildAutoChooser("4-note auto without exit")
        SmartDashboard.putData("auto choices", chooser)

        //<editor-fold desc="Alliance setup">
        try {
            alliance = DriverStation.getAlliance().get()
        } catch (e: Exception) {
            println(e)
            alliance = Alliance.Blue
        }
        invert = if (alliance == Alliance.Red) -1
        else 1
        //</editor-fold>
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


        //<editor-fold desc="Driver controller bindings">
        JoystickButton(driverXbox, Button.kA.value).onTrue((InstantCommand({ drivebase.zeroGyro() })))
        JoystickButton(driverXbox, Button.kX.value).whileTrue(aimAtLimelightCommand)
        JoystickButton(driverXbox, Button.kB.value).whileTrue(
            drivebase.pathfindThenFollowPath("New New Path")
                .alongWith(InstantCommand({ armSubsystem.moveArmToAngle(38.0) }, armSubsystem))
                .andThen(RepeatCommand(drivebase.driveCommand({ 0.0 }, { 0.0 }, { 1.0 }, { -0.95 })))
        )
        JoystickButton(driverXbox, Button.kLeftBumper.value).whileTrue(
            Commands.run({
                LimelightHelpers.setPipelineIndex(
                    "limelight-front",
                    1
                )
            }).alongWith(driveBotOrientedAngularVelocity).finallyDo(Runnable {
                LimelightHelpers.setPipelineIndex(
                    "limelight-front",
                    0
                )
            })
        )

        JoystickButton(driverXbox, Button.kY.value).whileTrue(aimAndPickUpNoteCommand)
        JoystickButton(
            driverXbox,
            Button.kStart.value
        ).onTrue(Commands.runOnce({ invert *= -1; SmartDashboard.putNumber("Invert", invert.toDouble()) }))
        JoystickButton(driverXbox, Button.kBack.value).onTrue(Commands.runOnce({
            drivebase.enableApriltags = !drivebase.enableApriltags
            SmartDashboard.putBoolean("AprilTags Enabled?", drivebase.enableApriltags)
            if (drivebase.enableApriltags) LimelightHelpers.setPipelineIndex("limelight-back", 0)
            if (!drivebase.enableApriltags) LimelightHelpers.setPipelineIndex("limelight-back", 1)

        }))

        JoystickButton(
            driverXbox,
            Button.kRightBumper.value
        ).whileTrue(AutoBuilder.buildAuto("Go to speaker and shoot"))

        //</editor-fold>
        //<editor-fold desc="Shooter controller bindings">
        JoystickButton(shooterXbox, Button.kLeftBumper.value).whileTrue(
            Commands.either(
                frostedFlakesCommand,
                intakeCommand
            ) { shooterRunning })
        JoystickButton(shooterXbox, Button.kRightBumper.value).whileTrue(reverseIntakeCommand)
        JoystickButton(shooterXbox, Button.kA.value).onTrue(homeArmCommand)
        JoystickButton(shooterXbox, Button.kB.value).whileTrue(overrideShootCommand)
        JoystickButton(shooterXbox, Button.kY.value).onTrue(moveArmToAmpCommand)
        JoystickButton(shooterXbox, Button.kX.value).onTrue(Commands.runOnce({
            CommandScheduler.getInstance().cancel(homeArmCommand, moveArmToAmpCommand, moveArmCommand)
        }))

        JoystickButton(shooterXbox, Button.kBack.value).whileTrue(
            RepeatCommand(
                drivebase.driveCommand(
                    { 0.0 },
                    { 0.0 },
                    { 0.0 },
                    { -1.0 })
            ).alongWith(Commands.run({ armSubsystem.moveArmToAngle(50.685) })
                .andThen(Commands.run({ shooter.setVelocity(2625.0, 2625.0) }))
            )
        )
        //</editor-fold>
        shooter.defaultCommand = shootCommand
        armSubsystem.defaultCommand = moveArmCommand

    }


    fun getAutonomousCommand(): Command {
        return chooser.selected
    }

    fun setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    fun setMotorBrake(brake: Boolean) {
        drivebase.setMotorBrake(brake)
    }

    companion object {
        var driverXbox: XboxController = XboxController(0)
        var shooterXbox: XboxController = XboxController(1)


        val rumbleShooterControllerTwiceNotifier: Notifier = Notifier {
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.5)
            Thread.sleep(125)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.0)
            // wait 0.25 seconds
            Thread.sleep(250)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.5)
            Thread.sleep(125)
            shooterXbox.setRumble(RumbleType.kBothRumble, 0.0)

        }

        var alliance: Alliance = Alliance.Blue

        var invert: Int = 1

        var hasNote: Boolean = false
    }

    fun executePov() {
        var pov = driverXbox.pov
        when (pov) {
            0 -> {
                CommandScheduler.getInstance().schedule(emoteCommand)
            }
            45 -> {
                // not assigned
            }
            90 -> {
                // not assigned
            }
            135 -> {
                // not assigned
            }
            180 -> {
                // not assigned
            }
            225 -> {
                // not assigned
            }
            270 -> {
                // not assigned
            }
            315 -> {
                // not assigned
            }
            -1 -> {
                CommandScheduler.getInstance().cancel(emoteCommand)
            }
        }


        pov = shooterXbox.pov
        when (pov) {
            0 -> {
                armSubsystem.moveArmToAngle(61.37)
            }
            45 -> {
                // not assigned
            }
            90 -> {
                // not assigned
            }
            135 -> {
                // not assigned
            }
            180 -> {
                // not assigned
            }
            225 -> {
                // not assigned
            }
            270 -> {
                // not assigned
            }
            315 -> {
                // not assigned
            }
            -1 -> {
                // not assigned
            }
        }
    }


}