package raidzero.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SwerveConstants {
    public static class Setpoints {
        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(1.151, 1.03, Rotation2d.fromDegrees(55)), // 12 Station
                new Pose2d(1.1383, 7.01, Rotation2d.fromDegrees(-55)) // 13 Station 1.0873
            )
        );

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(3.735, 3.14, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.06, 5.105, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9113, 2.93927, Rotation2d.fromDegrees(120)) // 22 Left
            )
        );

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
                new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Right
                new Pose2d(3.713, 4.925, Rotation2d.fromDegrees(300)), // 19 Right
                new Pose2d(4.9489, 5.16, Rotation2d.fromDegrees(240)), // 20 Right
                new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
                new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)) // 22 Right
            )
        );

        public static final List<Pose2d> ALL_REEFS = new ArrayList<Pose2d>() {
            {
                addAll(RIGHT_REEF_WAYPOINTS);
                addAll(LEFT_REEF_WAYPOINTS);
            }
        };

        public static final Pose2d BLUE_PROCESSOR = new Pose2d(5.987542, 0.78, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_PROCESSOR = new Pose2d(17.55 - 5.987542, 8.05 - 0.78, Rotation2d.fromDegrees(180));

        public static final Distance SETPOINT_TOLERANCE = Meters.of(0.05);
        public static final Angle ROTATION_ERROR_TOLERANCE = Degrees.of(1.0);
    }
}
