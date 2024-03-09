// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndPickUpNoteCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drivebase.AimAtLimelightCommand;
import frc.robot.commands.drivebase.AimAtTargetCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.util.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  private final Intake intake;
  private final Shooter shooter;
  private final Command intakeCommand, shootCommand, aimAtLimelightCommand, rotateCommand, driveToPoseCommand, aimAndPickUpNoteCommand;

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  XboxController shooterXbox = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivebase = SwerveSubsystem.getInstance();
    intake = new Intake(new IntakeIOSparkMax());
    shooter = new Shooter(new ShooterIOSparkFlex());
    intakeCommand = new IntakeCommand(intake, () -> MathUtil.applyDeadband(shooterXbox.getLeftTriggerAxis(), 0.1));
    shootCommand = new ShootCommand(shooter, () -> MathUtil.applyDeadband(shooterXbox.getRightTriggerAxis(), 0.1) * 3250);
    aimAtLimelightCommand = new AimAtLimelightCommand(drivebase, VisionSubsystem.getInstance());
    rotateCommand = new RotateCommand(drivebase);
    driveToPoseCommand = new ShootCommand(shooter, () -> 4000).alongWith(drivebase.driveToPose(new Pose2d(new Translation2d(2.720, 2.579), Rotation2d.fromDegrees(180))).andThen(drivebase.driveToPose(new Pose2d(new Translation2d(2.720, 2.579), Rotation2d.fromDegrees(180)))).andThen(new AimAtLimelightCommand(drivebase, VisionSubsystem.getInstance())).andThen(new IntakeCommand(intake, () -> 1)));
    aimAndPickUpNoteCommand = new AimAndPickUpNoteCommand(drivebase, VisionSubsystem.getInstance(), intake);
    int invert = getInvert();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(() -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * invert, () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * invert, () -> driverXbox.getRightX() * invert, () -> driverXbox.getRightY() * invert);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(() -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * invert, () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * invert, () -> driverXbox.getRawAxis(4) * invert);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(() -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * invert, () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * invert, () -> driverXbox.getRawAxis(2) * invert);

    Command sysIdForDrive = drivebase.sysIdDriveMotorCommand();
    Command sysIdForAngle = drivebase.sysIdAngleMotorCommand();

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    // Configure the trigger bindings
    configureBindings();
  }

  private int getInvert() {
    var alliance = DriverStation.getAlliance();
    int invert;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      invert = -1;
    } else {
      invert = 1;
    }


    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, () -> MathUtil.applyDeadband(driverXbox.getLeftY() * invert, OperatorConstants.LEFT_Y_DEADBAND), () -> MathUtil.applyDeadband(driverXbox.getLeftX() * invert, OperatorConstants.LEFT_X_DEADBAND), () -> MathUtil.applyDeadband(driverXbox.getRightX() * invert, OperatorConstants.RIGHT_X_DEADBAND), driverXbox::getYButtonPressed, driverXbox::getAButtonPressed, driverXbox::getXButtonPressed, driverXbox::getBButtonPressed);
    return invert;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, XboxController.Button.kX.value).whileTrue(aimAtLimelightCommand);
    new JoystickButton(driverXbox, 2).whileTrue(driveToPoseCommand);
    new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(rotateCommand);
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    //new JoystickButton(driverXbox, XboxController.Button.kStart.value).whileTrue(drivebase.sysIdAngleMotorCommand());
    //new JoystickButton(driverXbox, XboxController.Button.kBack.value).whileTrue(drivebase.sysIdDriveMotorCommand());


    new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value).whileTrue(intakeCommand); // independent of speed
    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).whileTrue(intakeCommand);
    new JoystickButton(driverXbox, XboxController.Button.kY.value).whileTrue(aimAndPickUpNoteCommand);

    shooter.setDefaultCommand(shootCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}