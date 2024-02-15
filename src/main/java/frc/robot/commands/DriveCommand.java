package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class DriveCommand extends Command {

  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;
  private final boolean isOpenLoop;
  private final SwerveController controller;
  private final Timer timer = new Timer();
  private final boolean headingCorrection;
  private double angle = 0;
  private double lastTime = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public DriveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                       BooleanSupplier driveMode) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.isOpenLoop = true;
    this.controller = swerve.getSwerveController();
    this.headingCorrection = true;
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (headingCorrection) {
      lastTime = timer.get();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3);
    double yVelocity = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    if (headingCorrection) {
      // Estimate the desired angle in radians.
      angle += (angVelocity * (timer.get() - lastTime)) * controller.config.maxAngularVelocity;
      // Get the desired ChassisSpeeds given the desired angle and current angle.
      ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVelocity, yVelocity, angle,
          swerve.getHeading().getRadians(), swerve.getMaximumSpeed());
      // Drive using given data points.
      swerve.getSwerveDrive().drive(
          SwerveController.getTranslation2d(correctedChassisSpeeds),
          correctedChassisSpeeds.omegaRadiansPerSecond,
          driveMode.getAsBoolean(),
          isOpenLoop);
      lastTime = timer.get();
    } else {
      // Drive relative to the direction of the robot (if the robot is facing 90
      // degrees, pressing forward on the stick drives toward 90 degrees)
      swerve.getSwerveDrive().drive(
          new Translation2d(xVelocity, yVelocity),
          angVelocity,
          false,
          isOpenLoop);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
