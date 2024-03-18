package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Constants;
import frc.robot.FieldConstants;

public class ShooterIOSparkFlex implements ShooterIO {
  // Calculated constants from Recalc (https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A555625%2C%22u%22%3A%22mm2%2Ag%22%7D&flywheelRadius=%7B%22s%22%3A50%2C%22u%22%3A%22mm%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A444.5%2C%22u%22%3A%22g%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%20Vortex%2A%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A8.3%2C%22u%22%3A%22oz%22%7D&shooterMomentOfInertia=%7B%22s%22%3A1.614%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A6783%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A0.807%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0):
  // using:
  // current limit: 40 A
  // efficiency: 100%
  // ratio: 1:1
  // target shooter rpm: 6783 (out of the max 6784)
  // projectile weight: 8.3 oz (from https://www.andymark.com/products/frc-2024-am-4999)
  // shooter radius: 2 in
  // shooter weight: 17.216 oz (4x 4.304 oz. 4 in. diameter wheels; durometer doesn't matter here https://www.andymark.com/products/4-in-compliant-wheels-options)
  // flywheel: {
  //  hex shaft: 6.043 oz (17.5 in; 0.345... oz/in)
  //  neo vortex: 444.5 g
  //  total: 615.8161682 g
  //  diameter: 50 mm
  //  ratio: 1:1
  // }
  //
  // this gives an estimated projectile speed of 45.40 ft/s

  // distance from shooter axle to center of shooting area: 12 1/8 in
  // distance from shooter axle to ground: 22 1/4 in

  private final CANSparkFlex lowerMotor = new CANSparkFlex(Constants.MotorConstants.LOWER_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex upperMotor = new CANSparkFlex(Constants.MotorConstants.UPPER_SHOOTER_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder lowerEncoder = lowerMotor.getEncoder();
  private final RelativeEncoder upperEncoder = upperMotor.getEncoder();
  private final SparkPIDController lowerPID = lowerMotor.getPIDController();
  private final SparkPIDController upperPID = upperMotor.getPIDController();
  public static final double P = Constants.ShooterConstants.P;
  public static final double I = Constants.ShooterConstants.I;
  public static final double D = Constants.ShooterConstants.D;


  public ShooterIOSparkFlex() {
    lowerEncoder.setPositionConversionFactor(1);
    lowerEncoder.setVelocityConversionFactor(1);
    upperEncoder.setPositionConversionFactor(1);
    upperEncoder.setVelocityConversionFactor(1);



    var i = new SimpleMotorFeedforward(0.028196, 0.0017838, 0.00020814);
    // use that to set the feed forward for the lower motor
    lowerPID.setFF(i.calculate(0));

    lowerPID.setP(1.0167e-7);
    lowerPID.setI(0);
    lowerPID.setD(0);

    upperMotor.follow(lowerMotor, true);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.lowerPositionDegrees = lowerEncoder.getPosition();
    inputs.lowerVelocityRpm = lowerEncoder.getVelocity();
    inputs.lowerAppliedVolts = lowerMotor.getAppliedOutput() * lowerMotor.getBusVoltage();
    // Theoretically,
    // I could implement a method to set surface speed and have that be the value the motors try to get to,
    // but I don't know if we would actually use that.
    // TODO: determine if we want to use surface speed
    inputs.lowerSurfaceSpeed = lowerEncoder.getVelocity() * Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI / 60.0 / 12.0; // feet per second
    inputs.lowerOutput = lowerMotor.get();
    inputs.lowerCurrentAmps = new double[] {lowerMotor.getOutputCurrent()};

    inputs.upperPositionDegrees = upperEncoder.getPosition();
    inputs.upperVelocityRpm = upperEncoder.getVelocity();
    inputs.upperAppliedVolts = upperMotor.getAppliedOutput() * upperMotor.getBusVoltage();
    inputs.upperSurfaceSpeed = upperEncoder.getVelocity() * Constants.ShooterConstants.WHEEL_DIAMETER * Math.PI / 60.0 / 12.0; // feet per second
    inputs.upperOutput = upperMotor.get();
    inputs.upperCurrentAmps = new double[] {upperMotor.getOutputCurrent()};
  }

  @Override
  public void set(double lowerPercent, double upperPercent) {
    lowerMotor.set(lowerPercent);
    upperMotor.set(upperPercent);
  }

  @Override
  public void setVelocity(double lowerRpm, double upperRpm) {
    lowerPID.setReference(lowerRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
    upperPID.setReference(upperRpm, CANSparkBase.ControlType.kSmartVelocity, 0);
  }

  @Override
  public void setVoltage(double volts) {
    lowerMotor.setVoltage(volts);
    upperMotor.setVoltage(volts);
  }


  // now for the calculation of the angle and velocity we'll need to shoot to make it into the speaker
//  public static double calculateAngle(Translation3d position) {
//    // we will use feet for this calculation
//    double x = Units.metersToFeet(position.getX());
//    double y = Units.metersToFeet(position.getY());
//
//    // from that, there is the problem of
//
//  }
//
//  public Translation3d calculateShooterDistanceFromGround(double angle) {
//    // this can be calculated using pretty basic trigonometry
//    double hypotenuse = 12.125; // inches
//    double opposite = Math.sin(angle) * hypotenuse;
//    double adjacent = Math.cos(angle) * hypotenuse;
//
//    // the center of the robot is 5.5 inches from the axle
//    double x = 5.5 - opposite;
//    double y = 22.25 - adjacent;
//  }
}
