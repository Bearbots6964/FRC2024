// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
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
import java.util.function.DoubleSupplier

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val drivebase: SwerveSubsystem = SwerveSubsystem.instance
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
    private val intakeCommand2: Command

    private val absoluteFieldDrive: Command

    /**
     *
     */
    private var driverXbox: XboxController = XboxController(0)
    /**
     *
     */
    private var shooterXbox: XboxController = XboxController(1)

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        intakeCommand = IntakeCommand(intake) { MathUtil.applyDeadband(shooterXbox.leftTriggerAxis, 0.1) }
        shootCommand = ShootCommand(shooter) { MathUtil.applyDeadband(shooterXbox.rightTriggerAxis, 0.1) * 5700 }
        aimAtLimelightCommand = AimAtLimelightCommand(drivebase, VisionSubsystem.instance)
        rotateCommand = RotateCommand(drivebase)
        driveToPoseCommand = ShootCommand(shooter, { 4000.0 }).alongWith(
            drivebase.driveToPose(
                Pose2d(
                    Translation2d(
                        2.720,
                        2.579
                    ), Rotation2d.fromDegrees(180.0)
                )
            ).andThen(drivebase.driveToPose(Pose2d(Translation2d(2.720, 2.579), Rotation2d.fromDegrees(180.0))))
                .andThen(AimAtLimelightCommand(drivebase, VisionSubsystem.instance))
                .andThen(IntakeCommand(intake, { 1.0 }))
        )
        aimAndPickUpNoteCommand = AimAndPickUpNoteCommand(drivebase, VisionSubsystem.instance, intake)
        armSysIdCommand = ArmSubsystem.generateSysIdCommand(armSubsystem.sysId, 2.0, 3.5, 1.5)
        moveArmCommand = MoveArmCommand { MathUtil.applyDeadband(shooterXbox.rightY, 0.1) }
        intakeCommand2 = IntakeCommand2(intake) { MathUtil.applyDeadband(shooterXbox.leftTriggerAxis, 0.1) }
        val invert = invert

        absoluteFieldDrive = AbsoluteFieldDrive(
            drivebase,
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * invert })
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        val driveFieldOrientedDirectAngle = drivebase.driveCommand(
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.rightX * invert },
            { driverXbox.rightY * invert })
        val closedAbsoluteDriveAdv = AbsoluteDriveAdv(
            drivebase,
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
        val driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            { -MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { -MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.getRawAxis(4) * invert })

        val driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) * invert },
            { MathUtil.applyDeadband(driverXbox.leftX, OperatorConstants.LEFT_X_DEADBAND) * invert },
            { driverXbox.getRawAxis(2) * invert })

        val sysIdForDrive = drivebase.sysIdDriveMotorCommand()
        val sysIdForAngle = drivebase.sysIdAngleMotorCommand()

        val driveFieldOrientedDirectAngleForTesting = drivebase.driveCommand(
            { MathUtil.applyDeadband(driverXbox.leftY, OperatorConstants.LEFT_Y_DEADBAND) },
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

        JoystickButton(driverXbox, 1).onTrue((InstantCommand({ drivebase.zeroGyro() })))
        JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(aimAtLimelightCommand)
        JoystickButton(driverXbox, 2).whileTrue(driveToPoseCommand)
        JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(rotateCommand)


        JoystickButton(
            shooterXbox,
            XboxController.Button.kLeftBumper.value
        ).whileTrue(intakeCommand) // independent of speed
        JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).whileTrue(intakeCommand)
        JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(aimAndPickUpNoteCommand)

        JoystickButton(shooterXbox, XboxController.Button.kA.value).whileTrue(moveArmCommand)
        JoystickButton(
            shooterXbox,
            XboxController.Button.kB.value
        ).whileTrue(shooter.generateSysIdCommand(shooter.sysIdRoutine, 2.0, 10.0, 10.0))

        JoystickButton(shooterXbox, XboxController.Button.kRightBumper.value).whileTrue(intakeCommand2)



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
}