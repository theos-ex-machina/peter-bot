package raidzero.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {
    public static class Arm {
        public static class ProximalJoint {
            public static final int MOTOR_ID = 10;
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
            public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

            public static final int CURRENT_LIMIT = 40;

            public static final double P = 1.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }

        public static class Positions {
            public static final double[] L4 = { 0.0, 0.0, 0.0 };
            public static final double[] L3 = { 0.0, 0.0, 0.0 };
            public static final double[] L2 = { 0.0, 0.0, 0.0 };
            public static final double[] L1 = { 0.0, 0.0, 0.0 };

            public static final double[] STATION = { 0.0, 0.0, 0.0 };
            public static final double[] GROUND_INTAKE = { 0.0, 0.0, 0.0 };

            public static final double[] L3_ALGAE = { 0.0, 0.0, 0.0 };
            public static final double[] L2_ALGAE = { 0.0, 0.0, 0.0 };
            public static final double[] BARGE = { 0.0, 0.0, 0.0 };
            public static final double[] PROCESSOR = { 0.0, 0.0, 0.0 };
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
        public static int L3_ALGAE = 1;
        public static int L2_ALGAE = 2;
    }
}