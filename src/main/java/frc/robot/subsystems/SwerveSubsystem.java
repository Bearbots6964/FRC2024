package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.trajectory.Trajectory;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {

  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.

  /**
   * The Singleton instance of this SwerveSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final SwerveSubsystem INSTANCE;

  private SwerveDrive swerveDrive;

  // TODO: Change this to be the actual maximum speed of the robot.
  private double maximumSpeed = Units.feetToMeters(14.5);

  // static INSTANCE = new SwerveSubsystem();
  static {
    try {
      INSTANCE = new SwerveSubsystem();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Returns the Singleton instance of this SwerveSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * SwerveSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static SwerveSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this SwerveSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private SwerveSubsystem() throws IOException {
    swerveDrive =
        new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
            .createSwerveDrive(Units.feetToMeters(14.5));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              new Translation2d(
                  Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                  Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
              Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
              true,
              false);
        });
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html
  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, headingX, headingY, getHeading().getRadians(), maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput, yInput, angle.getRadians(), getHeading().getRadians(), maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }
}
