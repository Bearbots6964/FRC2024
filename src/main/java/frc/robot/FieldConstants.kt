package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import java.io.IOException


/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. **All units in Meters** <br></br>
 * <br></br>
 *
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br></br>
 * <br></br>
 * Length refers to the *x* direction (as described by wpilib) <br></br>
 * Width refers to the *y* direction (as described by wpilib)
 */
object FieldConstants {
    /**
     *
     */
    val FIELD_LENGTH: Measure<Distance> = Units.Inches.of(651.223)
    /**
     *
     */
    val FIELD_WIDTH: Measure<Distance> = Units.Inches.of(323.277)
    /**
     *
     */
    val WING_X: Measure<Distance> = Units.Inches.of(229.201)
    /**
     *
     */
    val PODIUM_X: Measure<Distance> = Units.Inches.of(126.75)
    /**
     *
     */
    val STARTING_LINE_X: Measure<Distance> = Units.Inches.of(74.111)

    /**
     *
     */
    val AMP_CENTER: Translation2d = Translation2d(
        edu.wpi.first.math.util.Units.inchesToMeters(72.455),
        edu.wpi.first.math.util.Units.inchesToMeters(322.996)
    )

    /**
     *
     */
    val APRIL_TAG_WIDTH: Measure<Distance> = Units.Inches.of(6.50)
    /**
     *
     */
    private var APRIL_TAGS: AprilTagFieldLayout? = null

    init {
        try {
            APRIL_TAGS = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)
        } catch (e: IOException) {
            throw RuntimeException(e)
        }
    }

    /** Staging locations for each note  */
    object StagingLocations {
        /**
         *
         */
        private val CENTERLINE_X: Measure<Distance> = FIELD_LENGTH.divide(2.0)

        /**
         *
         */// need to update
        private val CENTERLINE_FIRST_Y: Measure<Distance> = Units.Inches.of(29.638)
        /**
         *
         */
        private val CENTERLINE_SEPARATION_Y: Measure<Distance> = Units.Inches.of(66.0)
        /**
         *
         */
        private val SPIKE_X: Measure<Distance> = Units.Inches.of(114.0)

        /**
         *
         */// need
        private val SPIKE_FIRST_Y: Measure<Distance> = Units.Inches.of(161.638)
        /**
         *
         */
        private val SPIKE_SEPARATION_Y: Measure<Distance> = Units.Inches.of(57.0)

        /**
         *
         */
        private val CENTERLINE_TRANSLATIONS: Array<Translation2d?> = arrayOfNulls(5)
        /**
         *
         */
        private val SPIKE_TRANSLATIONS: Array<Translation2d?> = arrayOfNulls(3)

        init {
            for (i in CENTERLINE_TRANSLATIONS.indices) {
                CENTERLINE_TRANSLATIONS[i] =
                    Translation2d(
                        CENTERLINE_X.`in`(Units.Inches),
                        CENTERLINE_FIRST_Y.`in`(Units.Inches) + (i * CENTERLINE_SEPARATION_Y.`in`(
                            Units.Inches
                        ))
                    )
            }
        }

        init {
            for (i in SPIKE_TRANSLATIONS.indices) {
                SPIKE_TRANSLATIONS[i] = Translation2d(
                    SPIKE_X.`in`(Units.Inches), SPIKE_FIRST_Y.`in`(Units.Inches) + (i * SPIKE_SEPARATION_Y.`in`(
                        Units.Inches
                    ))
                )
            }
        }
    }

    /** Each corner of the speaker *  */
    object Speaker {
        /**
         *
         */// corners (blue alliance origin)
        private val TOP_RIGHT_SPEAKER: Translation3d = Translation3d(
            edu.wpi.first.math.util.Units.inchesToMeters(18.055),
            edu.wpi.first.math.util.Units.inchesToMeters(238.815),
            edu.wpi.first.math.util.Units.inchesToMeters(83.091)
        )

        /**
         *
         */
        val TOP_LEFT_SPEAKER: Translation3d = Translation3d(
            edu.wpi.first.math.util.Units.inchesToMeters(18.055),
            edu.wpi.first.math.util.Units.inchesToMeters(197.765),
            edu.wpi.first.math.util.Units.inchesToMeters(83.091)
        )

        /**
         *
         */
        val BOTTOM_RIGHT_SPEAKER: Translation3d = Translation3d(
            0.0,
            edu.wpi.first.math.util.Units.inchesToMeters(238.815),
            edu.wpi.first.math.util.Units.inchesToMeters(78.324)
        )
        /**
         *
         */
        private val BOTTOM_LEFT_SPEAKER: Translation3d = Translation3d(
            0.0,
            edu.wpi.first.math.util.Units.inchesToMeters(197.765),
            edu.wpi.first.math.util.Units.inchesToMeters(78.324)
        )

        /** Center of the speaker opening (blue alliance)  */
        val CENTER_SPEAKER_OPENING: Translation3d = BOTTOM_LEFT_SPEAKER.interpolate(TOP_RIGHT_SPEAKER, 0.5)
    }
}