// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.windingmotor.util.math.AllianceFlipUtil;
import java.util.Optional;

public class LIB_ZoneConstants {

	public enum ZoneAngle {
		NONE(0),
		FORWARD(180),
		BACKWARD(0),
		RIGHT(-90),
		LEFT(90),
		SOURCE_RIGHT(53),
		SOURCE_LEFT(-55),
		REEF_BOTTOM_RIGHT(-120),
		REEF_BOTTOM_LEFT(120),
		REEF_BOTTOM(180),
		REEF_TOP_RIGHT(-60),
		REEF_TOP_LEFT(60),
		PROCESSOR(90),
		REEF_TOP(0);

		private final double angle;

		ZoneAngle(double angle) {
			this.angle = angle;
		}

		public double getAngle() {
			return angle;
		}

		public Rotation2d getRotation() {
			return Rotation2d.fromDegrees(angle);
		}
	}

	/**
	 * Gets the appropriate zone angle based on AprilTag ID and alliance color.
	 *
	 * @param id The AprilTag ID
	 * @return The ZoneAngle for the given tag ID
	 */
	public static ZoneAngle getZoneAngleForTagID(int id) {
		switch (id) {
			case 2:
			case 12:
				return ZoneAngle.SOURCE_RIGHT;

			case 1:
			case 13:
				return ZoneAngle.SOURCE_LEFT;

			case 7:
			case 18:
				return ZoneAngle.REEF_BOTTOM;

			case 6:
			case 19:
				return ZoneAngle.REEF_BOTTOM_LEFT;

			case 8:
			case 17:
				return ZoneAngle.REEF_BOTTOM_RIGHT;

			case 11:
			case 20:
				return ZoneAngle.REEF_TOP_LEFT;

			case 10:
			case 21:
				return ZoneAngle.REEF_TOP;

			case 9:
			case 22:
				return ZoneAngle.REEF_TOP_RIGHT;

			case 3:
			case 16:
				return ZoneAngle.PROCESSOR;

			default:
				return ZoneAngle.NONE;
		}
	}

	public enum ZonePose {
		NONE(new Translation2d(), ZoneAngle.NONE),
		FORWARD(new Translation2d(), ZoneAngle.FORWARD),
		BACKWARD(new Translation2d(), ZoneAngle.BACKWARD),
		RIGHT(new Translation2d(), ZoneAngle.RIGHT),
		LEFT(new Translation2d(), ZoneAngle.LEFT),
		PROCESSOR(new Translation2d(), ZoneAngle.PROCESSOR),

		// SOURCE
		SOURCE_RIGHT(new Translation2d(16.25, 7.25), ZoneAngle.SOURCE_RIGHT),
		SOURCE_LEFT(new Translation2d(16.8, 0.95), ZoneAngle.SOURCE_LEFT),

		// BOTTOM RIGHT
		REEF_BOTTOM_RIGHT_TOP(new Translation2d(13.559, 5.224), ZoneAngle.REEF_BOTTOM_RIGHT),
		REEF_BOTTOM_RIGHT_BOTTOM(new Translation2d(13.835, 5.055), ZoneAngle.REEF_BOTTOM_RIGHT),

		// BOTTOM LEFT
		REEF_BOTTOM_LEFT_TOP(new Translation2d(13.550, 2.856), ZoneAngle.REEF_BOTTOM_LEFT),
		REEF_BOTTOM_LEFT_BOTTOM(new Translation2d(13.856, 3), ZoneAngle.REEF_BOTTOM_LEFT),

		// BOTTOM
		REEF_BOTTOM_LEFT(new Translation2d(14.384, 3.852), ZoneAngle.REEF_BOTTOM),
		REEF_BOTTOM_RIGHT(new Translation2d(14.384, 4.181), ZoneAngle.REEF_BOTTOM),

		// TOP RIGHT
		REEF_TOP_RIGHT_BOTTOM(new Translation2d(12.558, 5.201), ZoneAngle.REEF_TOP_RIGHT),
		REEF_TOP_RIGHT_TOP(new Translation2d(12.279, 5.050), ZoneAngle.REEF_TOP_RIGHT),

		// TOP LEFT
		REEF_TOP_LEFT_TOP(new Translation2d(12.312, 3.052), ZoneAngle.REEF_TOP_LEFT),
		REEF_TOP_LEFT_BOTTOM(new Translation2d(12.559, 2.849), ZoneAngle.REEF_TOP_LEFT),

		// TOP
		REEF_TOP_LEFT(new Translation2d(11.78, 3.871), ZoneAngle.REEF_TOP),
		REEF_TOP_RIGHT(new Translation2d(11.78, 4.195), ZoneAngle.REEF_TOP);

		private final Translation2d translation;
		private final ZoneAngle zoneAngle;

		ZonePose(Translation2d translation, ZoneAngle zoneAngle) {
			this.translation = translation;
			this.zoneAngle = zoneAngle;
		}

		public Translation2d getTranslation() {
			return translation;
		}

		public ZoneAngle getZoneAngle() {
			return zoneAngle;
		}

		public Pose2d getPose() {
			return new Pose2d(translation, zoneAngle.getRotation());
		}

		public Optional<Pose2d> getPoseForAlliance(boolean isRed) {

			// If red
			if (isRed) {
				// Red
				return Optional.of(new Pose2d(translation, zoneAngle.getRotation()));
			}

			// If not red aka blue
			if (!isRed) {
				// Blue
				Pose2d flippedPose = AllianceFlipUtil.apply(getPose());

				Rotation2d targetRotation =
						Rotation2d.fromDegrees(flippedPose.getRotation().getDegrees() - 0);

				Pose2d rotatedPose = new Pose2d(flippedPose.getTranslation(), targetRotation);

				return Optional.of(new Pose2d(rotatedPose.getTranslation(), rotatedPose.getRotation()));
			}

			return Optional.empty();
		}
	}
}
