package raidzero.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.robot.subsystems.drivetrain.TunerConstants;

public class Constants {
    public static class Arm {
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

            public static final double LENGTH = 1.0;
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

            public static final double LENGTH = 1.0;
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
        }

        public static class Positions {
            public static final double[] L4 = { -0.25, 1.84, 0.875 };
            public static final double[] L3 = { -0.25, 1.22, 0.875 };
            public static final double[] L2 = { -0.25, 0.82, 0.875 };
            public static final double[] L1 = { -0.30, 0.47, 0.875 };

            public static final double[] STATION = { -0.35, 0.95, 0.125 };
            public static final double[] GROUND_INTAKE = { -0.8, -0.1, 0.9 };

            public static final double[] L3_ALGAE = { -0.3, 1.32, 0.7 };
            public static final double[] L2_ALGAE = { -0.3, 0.92, 0.75 };
            public static final double[] BARGE = { 0.0, 2.0, 0.75 };
            public static final double[] PROCESSOR = { 0.35, 0.0, 0.8 };
        }
    }

    public static class Bindings {
        public static final int OPERATOR_PORT = 1;
        public static CommandGenericHID operator = new CommandGenericHID(OPERATOR_PORT);

        // same as before
        public static Trigger L1 = operator.button(6);
        public static Trigger L2 = operator.button(7);
        public static Trigger L3 = operator.button(8);
        public static Trigger L4 = operator.button(9);

        public static Trigger STATION = operator.button(12); // "home" labelled button
        public static Trigger GROUND_INTAKE = operator.button(14); // Bottom right

        public static Trigger PROCESSOR = operator.button(15); // bottom left
        public static Trigger BARGE = operator.button(16); // top left

        // Same as before
        public static Trigger CORAL_INTAKE = operator.button(11);
        public static Trigger CORAL_EXTAKE = operator.button(10);

        // Same as before
        public static Trigger ALGAE_INTAKE = operator.button(4);
        public static Trigger ALGAE_EXTAKE = operator.button(5);

        // Axis buttons next to algae shit
        public static Trigger L3_ALGAE = operator.axisGreaterThan(0, 0.5);
        public static Trigger L2_ALGAE = operator.axisGreaterThan(1, 0.5);
    }

    public static class Simulation {
        public static final double BOT_WEIGHT_LBS = 90.0;

        public static final double TRACK_WIDTH_IN = 18.5;
        public static final double TRACK_LENGTH_IN = 18.5;

        public static final double BOT_LENGTH_BUMPERS = 24;
        public static final double BOT_WIDTH_BUMPERS = 24;

        public static final double WHEEL_COF = 1.2;
        public static final double SIM_LOOP_PERIOD_S = 0.002;

        public static final DriveTrainSimulationConfig SWERVE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.kDriveGearRatio,
                    TunerConstants.kSteerGearRatio,
                    TunerConstants.kDriveFrictionVoltage,
                    TunerConstants.kSteerFrictionVoltage,
                    TunerConstants.kWheelRadius,
                    TunerConstants.kSteerInertia,
                    WHEEL_COF
                )
            )
            .withTrackLengthTrackWidth(Inches.of(TRACK_LENGTH_IN), Inches.of(TRACK_WIDTH_IN))
            .withBumperSize(Inches.of(BOT_LENGTH_BUMPERS), Inches.of(BOT_WIDTH_BUMPERS));
    }
}