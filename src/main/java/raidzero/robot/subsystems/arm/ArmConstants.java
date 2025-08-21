package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import java.util.List;
import raidzero.lib.wrappers.motors.LazyMotor.MotorTelemetry;

public class ArmConstants {
    public static class ProximalJoint {
        public static final int MOTOR_ID = 10;
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final double STATOR_CURRENT_LIMIT = 40;
        public static final double SUPPLY_CURRENT_LIMIT = 50;

        public static final double FORWARD_SOFT_LIMIT = 0.45;
        public static final double REVERSE_SOFT_LIMIT = 0.05;

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final double CRUISE_VELOCITY = 1.0;
        public static final double ACCELERATION = 1.0;

        public static final Distance LENGTH = Meters.of(1.0);
        public static final Mass MASS = Kilograms.of(1.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class DistalJoint {
        public static final int MOTOR_ID = 11;
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final double STATOR_CURRENT_LIMIT = 40;
        public static final double SUPPLY_CURRENT_LIMIT = 50;

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final double CRUISE_VELOCITY = 1.0;
        public static final double ACCELERATION = 1.0;

        public static final Distance LENGTH = Meters.of(1.0);
        public static final Mass MASS = Kilograms.of(1.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Wrist {
        public static final int MOTOR_ID = 13;
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO550_JST;
        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

        public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final int STATOR_CURRENT_LIMIT = 40;
        public static final int SUPPLY_CURRENT_LIMIT = 50;

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final double CRUISE_VELOCITY = 1.0;
        public static final double ACCELERATION = 1.0;

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Intake {
        public static final int LEADER_ID = 14;
        public static final int FOLLOWER_ID = 15;
        public static final boolean FOLLOWER_INVERTED = false;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

        public static final int BOTTOM_LASERCAN_ID = 0;
        public static final int BOTTOM_LASER_THRESHOLD_MM = 50;

        public static final int TOP_LASERCAN_ID = 1;
        public static final int TOP_LASER_THRESHOLD_MM = 50;

        public static final double INTAKE_SPEED = 1.0;
        public static final double EXTAKE_SPEED = 1.0;

        public static final double ALGAE_EJECT_SPEED = 1.0;
        public static final double ALGAE_HOLD_SPEED = 0.4;

        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;
        public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final int STATOR_CURRENT_LIMIT = 40;
        public static final int SUPPLY_CURRENT_LIMIT = 50;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 0.0;

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Positions {
        public static final Pose2d L4 = new Pose2d(0.25, 1.84, Rotation2d.kZero);
        public static final Angle L4_WRIST_ANGLE = Rotations.of(0.875);
        public static final List<Pose2d> L4_INTERPOLATION_PATH = List.of(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new Pose2d(0.1, 1.22, Rotation2d.kZero),
            new Pose2d(-0.1, 1.84, Rotation2d.kZero),
            L4
        );

        public static final Pose2d L3 = new Pose2d(0.25, 1.22, Rotation2d.kZero);
        public static final Angle L3_WRIST_ANGLE = Rotations.of(0.875);

        public static final Pose2d L2 = new Pose2d(0.25, 0.82, Rotation2d.kZero);
        public static final Angle L2_WRIST_ANGLE = Rotations.of(0.875);

        public static final Pose2d L1 = new Pose2d(0.30, 0.47, Rotation2d.kZero);
        public static final Angle L1_WRIST_ANGLE = Rotations.of(0.875);

        public static final Pose2d STATION = new Pose2d(0.35, 0.95, Rotation2d.kZero);
        public static final Angle STATION_WRIST_ANGLE = Rotations.of(0.125);

        public static final Pose2d GROUND_INTAKE = new Pose2d(0.8, -0.1, Rotation2d.kZero);
        public static final Angle GROUND_INTAKE_WRIST_ANGLE = Rotations.of(0.9);

        public static final Pose2d L3_ALGAE = new Pose2d(0.3, 1.32, Rotation2d.kZero);
        public static final Angle L3_ALGAE_WRIST_ANGLE = Rotations.of(0.7);

        public static final Pose2d L2_ALGAE = new Pose2d(0.3, 0.92, Rotation2d.kZero);
        public static final Angle L2_ALGAE_WRIST_ANGLE = Rotations.of(0.75);

        public static final Pose2d BARGE = new Pose2d(0.0, 2.0, Rotation2d.kZero);
        public static final Angle BARGE_WRIST_ANGLE = Rotations.of(0.75);

        public static final Pose2d PROCESSOR = new Pose2d(-0.35, 0.0, Rotation2d.kZero);
        public static final Angle PROCESSOR_WRIST_ANGLE = Rotations.of(0.8);

        public static final Angle POSITION_TOLERANCE = Degrees.of(0.2);
    }
}
