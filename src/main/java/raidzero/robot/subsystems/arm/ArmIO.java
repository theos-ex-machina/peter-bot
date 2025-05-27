package raidzero.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyFXS;

import static raidzero.robot.subsystems.arm.ArmConstants.ProximalJoint;
import static raidzero.robot.subsystems.arm.ArmConstants.DistalJoint;
import static raidzero.robot.subsystems.arm.ArmConstants.Wrist;;

public interface ArmIO extends SubsystemIO {
    /**
     * Moves both joints of the arm to the specified angle
     * 
     * @param proximalSetpoint the proximal joint setpoint 
     * @param distalSeptoint the distal joint setpoint
     */
    void moveJoints(Angle proximalSetpoint, Angle distalSeptoint);

    /**
     * Gets the angles of the jonts
     * 
     * @return the angle of the joints. index 0 is the proximal joint, and index 1 is the distal
     */
    Angle[] getJointAngles();

    /**
     * Moves the wrist to thee specified setpoint
     * 
     * @param setpoint the wrist setpoint
     */
    void moveWrist(Angle setpoint);

    /**
     * Gets the angle of the wrist
    
     * @return the angle of the wrist
     */
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

        @Override
        public void updateTelemetry() {
            proximalJoint.updateTelemetry(ProximalJoint.TELEMETRY);
            distalJoint.updateTelemetry(DistalJoint.TELEMETRY);
            wrist.updateTelemetry(Wrist.TELEMETRY);
        }
    }

    public class Sim extends Real {}
}
