package raidzero.robot.subsystems.arm;

import raidzero.lib.LazyFXS;
import raidzero.robot.Constants.Arm.DistalJoint;
import raidzero.robot.Constants.Arm.ProximalJoint;
import raidzero.robot.Constants.Arm.Wrist;

public interface ArmIO {
    void moveWithRotations(double proximalSetpoint, double distalSeptoint);

    void moveWrist(double setpoint);

    public class Real implements ArmIO {
        private LazyFXS proximalJoint, distalJoint;
        private LazyFXS wrist;

        public Real() {
            proximalJoint = new LazyFXS(
                ProximalJoint.MOTOR_ID,
                ProximalJoint.MOTOR_ARRANGEMENT,
                ProximalJoint.SENSOR_TO_MECHANISM_RATIO,
                ProximalJoint.INVERTED,
                ProximalJoint.STATOR_CURRENT_LIMIT,
                ProximalJoint.SUPPLY_CURRENT_LIMIT
            ).withSoftLimits(
                true, ProximalJoint.FORWARD_SOFT_LIMIT,
                true, ProximalJoint.REVERSE_SOFT_LIMIT
            ).withMotionMagicConfiguration(
                ProximalJoint.P, ProximalJoint.I, ProximalJoint.D,
                ProximalJoint.S, ProximalJoint.G, ProximalJoint.V, ProximalJoint.A,
                ProximalJoint.GRAVITY_TYPE,
                ProximalJoint.CRUISE_VELOCITY, ProximalJoint.ACCELERATION
            ).build();

            distalJoint = new LazyFXS(
                DistalJoint.MOTOR_ID,
                DistalJoint.MOTOR_ARRANGEMENT,
                DistalJoint.SENSOR_TO_MECHANISM_RATIO,
                DistalJoint.INVERTED,
                DistalJoint.STATOR_CURRENT_LIMIT,
                DistalJoint.SUPPLY_CURRENT_LIMIT
            ).withMotionMagicConfiguration(
                DistalJoint.P, DistalJoint.I, DistalJoint.D,
                DistalJoint.S, DistalJoint.G, DistalJoint.V, DistalJoint.A,
                DistalJoint.GRAVITY_TYPE,
                DistalJoint.CRUISE_VELOCITY, DistalJoint.ACCELERATION
            ).build();

            wrist = new LazyFXS(
                Wrist.MOTOR_ID, Wrist.MOTOR_ARRANGEMENT, Wrist.SENSOR_TO_MECHANISM_RATIO, Wrist.INVERTED_VALUE, Wrist.STATOR_CURRENT_LIMIT,
                Wrist.SUPPLY_CURRENT_LIMIT
            ).withMotionMagicConfiguration(
                Wrist.P, Wrist.I, Wrist.D,
                Wrist.S, Wrist.G, Wrist.V, Wrist.A,
                Wrist.GRAVITY_TYPE,
                Wrist.CRUISE_VELOCITY, Wrist.ACCELERATION
            ).build();
        }

        @Override
        public void moveWithRotations(double proximalSetpoint, double distalSeptoint) {
            proximalJoint.moveTo(proximalSetpoint);
            distalJoint.moveTo(distalSeptoint);
        }

        @Override
        public void moveWrist(double setpoint) {
            wrist.moveTo(setpoint);
        }
    }

    public class Sim implements ArmIO {
        public Sim() {

        }

        @Override
        public void moveWithRotations(double proximalSetpoint, double distalSeptoint) {
            // TODO Auto-generated method stub
        }

        @Override
        public void moveWrist(double setpoint) {
            // TODO Auto-generated method stub
        }
    }
}
