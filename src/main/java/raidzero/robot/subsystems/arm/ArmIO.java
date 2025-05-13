package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSSimState;
import edu.wpi.first.units.measure.Angle;
import raidzero.lib.wrappers.LazyFXS;
import raidzero.robot.Constants.Arm.DistalJoint;
import raidzero.robot.Constants.Arm.ProximalJoint;
import raidzero.robot.Constants.Arm.Wrist;

public interface ArmIO {
    void moveJoints(Angle proximalSetpoint, Angle distalSeptoint);
    Angle[] getJointAngles();

    void moveWrist(Angle setpoint);
    Angle getWristAngle();

    public class Real implements ArmIO {
        protected LazyFXS proximalJoint, distalJoint;
        protected LazyFXS wrist;

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
        public void moveJoints(Angle proximalSetpoint, Angle distalSeptoint) {
            proximalJoint.moveTo(proximalSetpoint);
            distalJoint.moveTo(distalSeptoint);
        }

        @Override
        public Angle[] getJointAngles() {
            return new Angle[] { proximalJoint.getFeedbackPosition(), distalJoint.getFeedbackPosition() };
        }

        @Override
        public void moveWrist(Angle setpoint) {
            wrist.moveTo(setpoint);
        }

        @Override
        public Angle getWristAngle() {
            return wrist.getFeedbackPosition();
        }
    }

    public class Sim extends Real {
        TalonFXSSimState proximalJointSim, distalJointSim;
        TalonFXSSimState wristSim;

        public Sim() {
            super();

            proximalJointSim = super.proximalJoint.getMotor().getSimState();
            proximalJointSim.setSupplyVoltage(Volts.of(12));

            distalJointSim = super.distalJoint.getMotor().getSimState();
            distalJointSim.setSupplyVoltage(Volts.of(12));

            wristSim = super.wrist.getMotor().getSimState();
            wristSim.setSupplyVoltage(12);
        }

        @Override
        public void moveJoints(Angle proximalSetpoint, Angle distalSeptoint) {
            proximalJointSim.setRawRotorPosition(proximalSetpoint);
            distalJointSim.setRawRotorPosition(distalSeptoint);
        }

        @Override
        public void moveWrist(Angle setpoint) {
            wristSim.setRawRotorPosition(setpoint);
        }
    }
}
