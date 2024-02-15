// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  CommandJoystick joystick = new CommandJoystick(0);

  private boolean fieldRelative = true;

  Command driveCommand = new DriveCommand(swerveSubsystem, () -> MathUtil.applyDeadband(joystick.getX(), Constants.OperatorConstants.JOYSTICK_X_DEADBAND), () -> MathUtil.applyDeadband(joystick.getY(), Constants.OperatorConstants.JOYSTICK_Y_DEADBAND), () -> MathUtil.applyDeadband(joystick.getTwist(), Constants.OperatorConstants.JOYSTICK_TWIST_DEADBAND), () -> fieldRelative);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
