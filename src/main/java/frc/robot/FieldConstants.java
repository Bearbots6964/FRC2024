package frc.robot;


import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import java.io.IOException;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final Measure<Distance> FIELD_LENGTH = Inches.of(651.223);
  public static final Measure<Distance> FIELD_WIDTH = Inches.of(323.277);
  public static final Measure<Distance> WING_X = Inches.of(229.201);
  public static final Measure<Distance> PODIUM_X = Inches.of(126.75);
  public static final Measure<Distance> STARTING_LINE_X = Inches.of(74.111);

  public static final Translation2d AMP_CENTER =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final Measure<Distance> CENTERLINE_X = FIELD_LENGTH.divide(2.0);

    // need to update
    public static final Measure<Distance> CENTERLINE_FIRST_Y = Inches.of(29.638);
    public static final Measure<Distance> CENTERLINE_SEPARATION_Y = Inches.of(66);
    public static final Measure<Distance> SPIKE_X = Inches.of(114);
    // need
    public static final Measure<Distance> SPIKE_FIRST_Y = Inches.of(161.638);
    public static final Measure<Distance> SPIKE_SEPARATION_Y = Inches.of(57);

    public static final Translation2d[] CENTERLINE_TRANSLATIONS = new Translation2d[5];
    public static final Translation2d[] SPIKE_TRANSLATIONS = new Translation2d[3];

    static {
      for (int i = 0; i < CENTERLINE_TRANSLATIONS.length; i++) {
        CENTERLINE_TRANSLATIONS[i] =
            new Translation2d(CENTERLINE_X.in(Inches), CENTERLINE_FIRST_Y.in(Inches) + (i * CENTERLINE_SEPARATION_Y.in(Inches)));
      }
    }

    static {
      for (int i = 0; i < SPIKE_TRANSLATIONS.length; i++) {
        SPIKE_TRANSLATIONS[i] = new Translation2d(SPIKE_X.in(Inches), SPIKE_FIRST_Y.in(Inches) + (i * SPIKE_SEPARATION_Y.in(Inches)));
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static final Translation3d TOP_RIGHT_SPEAKER =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static final Translation3d TOP_LEFT_SPEAKER =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d BOTTOM_RIGHT_SPEAKER =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d BOTTOM_LEFT_SPEAKER =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d CENTER_SPEAKER_OPENING =
        BOTTOM_LEFT_SPEAKER.interpolate(TOP_RIGHT_SPEAKER, 0.5);
  }

  public static final Measure<Distance> APRIL_TAG_WIDTH = Inches.of(6.50);
  public static final AprilTagFieldLayout APRIL_TAGS;

  static {
    try {
      APRIL_TAGS = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}