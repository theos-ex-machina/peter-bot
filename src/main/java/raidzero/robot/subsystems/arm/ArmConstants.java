package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import raidzero.lib.wrappers.motors.LazyMotor.MotorTelemetry;

public class ArmConstants {
    public static class ProximalJoint {
        public static final int MOTOR_ID = 10;
        public static final int FOLLOWER_ID = 13;
        public static final boolean FOLLOWER_INVERTED = false;

        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = 93.6;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 100.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.45);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0.05);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(3.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(3.0);

        public static final Distance LENGTH = Meters.of(1.0);
        public static final Distance GROUND_TO_AXIS = Meters.of(0.25);
        public static final Mass MASS = Kilograms.of(1.0);

        public static final Angle STARTING_ANGLE = Rotations.of(0.25);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class DistalJoint {
        public static final int MOTOR_ID = 12;
        public static final int FOLLOWER_ID = 11;
        public static final boolean FOLLOWER_INVERTED = false;

        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = 150.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 100;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.9;
        public static final double G = 0.0;
        public static final double V = 0.0;// 3.0;
        public static final double A = 0.0;// 1.5;

        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(1.25);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0.25);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(3.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(3.0);

        public static final Distance LENGTH = Meters.of(0.85);
        public static final Mass MASS = Kilograms.of(1.0);

        public static final Angle STARTING_ANGLE = Rotations.of(0.75);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Wrist {
        public static final int MOTOR_ID = 14;
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.NEO550_JST;
        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

        public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 80.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;
        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(1.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(1.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Intake {
        public static final int LEADER_ID = 15;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

        public static final int BOTTOM_LASERCAN_ID = 0;
        public static final Distance BOTTOM_LASER_THRESHOLD_MM = Millimeters.of(50);

        public static final int TOP_LASERCAN_ID = 1;
        public static final Distance TOP_LASER_THRESHOLD_MM = Millimeters.of(50);

        public static final double INTAKE_SPEED_PERCENT = 1.0;
        public static final double EXTAKE_SPEED_PERCENT = 1.0;

        public static final double ALGAE_EJECT_SPEED_PERCENT = 1.0;
        public static final double ALGAE_HOLD_SPEED_PERCENT = 0.4;

        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;
        public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static class Positions {
        private static final Distance HOME_X = Meters.of(0);
        private static final Distance HOME_Y = Meters.of(0.4);
        private static final Angle HOME_WRIST_ANGLE = Rotations.of(0);
        private static final boolean HOME_THETA1_UP = false;

        public static final ArmPose HOME = new ArmPose(HOME_X, HOME_Y, HOME_WRIST_ANGLE, HOME_THETA1_UP);

        private static final Distance L4_X = Meters.of(0.40);
        private static final Distance L4_Y = Meters.of(1.84);
        private static final Angle L4_WRIST_ANGLE = Rotations.of(0.875);
        private static final boolean L4_THETA1_UP = false;

        public static final ArmPose L4 = new ArmPose(L4_X, L4_Y, L4_WRIST_ANGLE, L4_THETA1_UP);

        private static final Distance L3_X = Meters.of(0.40);
        private static final Distance L3_Y = Meters.of(1.21);
        private static final Angle L3_WRIST_ANGLE = Rotations.of(0.875);
        private static final boolean L3_THETA1_UP = false;

        public static final ArmPose L3 = new ArmPose(L3_X, L3_Y, L3_WRIST_ANGLE, L3_THETA1_UP);

        private static final Distance L2_X = Meters.of(0.40);
        private static final Distance L2_Y = Meters.of(0.81);
        private static final Angle L2_WRIST_ANGLE = Rotations.of(0.875);
        private static final boolean L2_THETA1_UP = false;

        public static final ArmPose L2 = new ArmPose(L2_X, L2_Y, L2_WRIST_ANGLE, L2_THETA1_UP);

        private static final Distance L1_X = Meters.of(0.40);
        private static final Distance L1_Y = Meters.of(0.46);
        private static final Angle L1_WRIST_ANGLE = Rotations.of(0.875);
        private static final boolean L1_THETA1_UP = false;

        public static final ArmPose L1 = new ArmPose(L1_X, L1_Y, L1_WRIST_ANGLE, L1_THETA1_UP);

        // ! poses are completely unverified.
        private static final Distance STATION_X = Meters.of(0.35);
        private static final Distance STATION_Y = Meters.of(0.95);
        private static final Angle STATION_WRIST_ANGLE = Rotations.of(0.125);
        private static final boolean STATION_THETA1_UP = false;

        public static final ArmPose STATION = new ArmPose(STATION_X, STATION_Y, STATION_WRIST_ANGLE, STATION_THETA1_UP);

        private static final Distance GROUND_INTAKE_X = Meters.of(0.8);
        private static final Distance GROUND_INTAKE_Y = Meters.of(-0.1);
        private static final Angle GROUND_INTAKE_WRIST_ANGLE = Rotations.of(0.9);
        private static final boolean GROUND_INTAKE_THETA1_UP = false;

        public static final ArmPose GROUND_INTAKE = new ArmPose(
            GROUND_INTAKE_X, GROUND_INTAKE_Y, GROUND_INTAKE_WRIST_ANGLE, GROUND_INTAKE_THETA1_UP
        );

        private static final Distance L3_ALGAE_X = Meters.of(0.3);
        private static final Distance L3_ALGAE_Y = Meters.of(1.32);
        private static final Angle L3_ALGAE_WRIST_ANGLE = Rotations.of(0.875);
        private static final boolean L3_ALGAE_THETA1_UP = false;

        public static final ArmPose L3_ALGAE = new ArmPose(L3_ALGAE_X, L3_ALGAE_Y, L3_ALGAE_WRIST_ANGLE, L3_ALGAE_THETA1_UP);

        private static final Distance L2_ALGAE_X = Meters.of(0.3);
        private static final Distance L2_ALGAE_Y = Meters.of(0.92);
        private static final Angle L2_ALGAE_WRIST_ANGLE = Rotations.of(0.75);
        private static final boolean L2_ALGAE_THETA1_UP = false;

        public static final ArmPose L2_ALGAE = new ArmPose(L2_ALGAE_X, L2_ALGAE_Y, L2_ALGAE_WRIST_ANGLE, L2_ALGAE_THETA1_UP);

        private static final Distance BARGE_X = Meters.of(0.0);
        private static final Distance BARGE_Y = Meters.of(2.0);
        private static final Angle BARGE_WRIST_ANGLE = Rotations.of(0.75);
        private static final boolean BARGE_THETA1_UP = false;

        public static final ArmPose BARGE = new ArmPose(BARGE_X, BARGE_Y, BARGE_WRIST_ANGLE, BARGE_THETA1_UP);

        private static final Distance PROCESSOR_X = Meters.of(-0.35);
        private static final Distance PROCESSOR_Y = Meters.of(0.0);
        private static final Angle PROCESSOR_WRIST_ANGLE = Rotations.of(0.8);
        private static final boolean PROCESSOR_THETA1_UP = false;

        public static final ArmPose PROCESSOR = new ArmPose(PROCESSOR_X, PROCESSOR_Y, PROCESSOR_WRIST_ANGLE, PROCESSOR_THETA1_UP);

        public static final Angle POSITION_TOLERANCE_ANGLE = Degrees.of(1.0);
        public static final Distance POSITION_TOLERANCE_DISTANCE = Meters.of(0.05);
    }

    public static class ArmPose {
        Pose2d pose;
        Angle wristAngle;
        boolean theta1Up;
        List<ArmPose> interpolationPath = null;

        public ArmPose(Distance x, Distance y, Angle wristAngle, boolean theta1Up) {
            this.pose = new Pose2d(x.in(Meters), y.in(Meters), Rotation2d.kZero);
            this.wristAngle = wristAngle;
            this.theta1Up = theta1Up;
        }

        public ArmPose withInterPolationPath(Pose2d... poses) {
            this.interpolationPath = List.of();
            return this;
        }
    }
}
