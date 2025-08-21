package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static raidzero.robot.subsystems.arm.ArmConstants.Intake.*;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.LazyCan;
import raidzero.lib.wrappers.motors.LazyFXS;

public interface IntakeIO extends SubsystemIO {
    /**
     * Sets the speed of the intake rollers
     *
     * @param speed the speed to set in percentage of full speed [0, 1]
    */
    public void setSpeed(double speed);

    /**
     * Stops the intake rollers
     */
    public void stop();

    /**
     * Gets the bottom laser sensor distance
     *
     * @return the distance in mm
     */
    public int getBottomLaserDistance();

    /**
     * Returns true if the bottom laser is within the threshold

     * @return a trigger that is activated if the bottom laser is within the specified setpoint
     */
    public Trigger bottomLaserWithinThreshold();

    /**
     * Gets the top Laser sensor distance
     *
     * @return the distance in mm
     */
    public int getTopLaserDistance();

    /**
     * Returns true if the bottom laser is within the threshold

     * @return a trigger that is activated if the top laser is within the specified setpoint
     */
    public Trigger topLaserWithinThreshold();

    public class Real implements IntakeIO {
        private LazyFXS roller;
        private LazyCan bottomLaser, topLaser;

        public Real() {
            roller = new LazyFXS(
                LEADER_ID, 
                MOTOR_ARRANGEMENT,
                SENSOR_TO_MECHANISM_RATIO, 
                INVERTED_VALUE,
                STATOR_CURRENT_LIMIT.in(Amps), 
                SUPPLY_CURRENT_LIMIT.in(Amps)
            ).build();

            bottomLaser = new LazyCan(BOTTOM_LASERCAN_ID).withRangingMode(RangingMode.SHORT)
                .withRegionOfInterest(14, 8, 16, 16).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
                .withThreshold(BOTTOM_LASER_THRESHOLD_MM.in(Millimeters));

            topLaser = new LazyCan(TOP_LASERCAN_ID).withRangingMode(RangingMode.LONG)
                .withRegionOfInterest(8, 14, 16, 4).withTimingBudget(TimingBudget.TIMING_BUDGET_20MS)
                .withThreshold(TOP_LASER_THRESHOLD_MM.in(Millimeters));
        }

        @Override
        public void setSpeed(double speed) {
            roller.set(speed);
        }

        @Override
        public void stop() {
            roller.stop();
        }

        @Override
        public int getBottomLaserDistance() {
            return bottomLaser.getDistanceMm();
        }

        @Override
        public Trigger bottomLaserWithinThreshold() {
            return new Trigger(() -> bottomLaser.withinThreshold());
        }

        @Override
        public int getTopLaserDistance() {
            return topLaser.getDistanceMm();
        }

        @Override
        public Trigger topLaserWithinThreshold() {
            return new Trigger(() -> topLaser.withinThreshold());
        }

        @Override
        public void updateTelemetry() {
            roller.updateTelemetry(TELEMETRY);
        }
    }

    public class Sim extends Real {}
}
