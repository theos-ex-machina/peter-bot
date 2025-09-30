package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import raidzero.lib.Interpolate;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.arm.ArmConstants.DistalJoint;
import raidzero.robot.subsystems.arm.ArmConstants.Positions;
import raidzero.robot.subsystems.arm.ArmConstants.ProximalJoint;

public class Arm extends R0Subsystem<ArmIO> {
    private static Arm system;

    /**
     * Constructs a {@link Arm} subsystem instance
     *
     * @param io the hardware IO to use (simulation or real hardware)
     */
    private Arm(ArmIO io) {
        super(io);
    }

    /**
     * Moves the arm to the supplied setpoint with the supplied wrist angle
     *
     * @param setpoint the goal position of the arm
     * @param wristAngle the angle of the wrist relative to the bot
     * @return a {@link Command}
     */
    public Command moveTo(Pose2d setpoint, Angle wristAngle, boolean configuration1) {
        return run(() -> {
            Angle[] angles = calculateJointAngles(setpoint, configuration1);

            io.moveJoints(angles[0], angles[1]);
            io.moveWrist(wristAngle.minus(angles[0].plus(angles[1])));
        });
    }

    public Command moveTo(Angle proximalSetpoint, Angle distalSetpoint) {
        return run(() -> {
            io.moveJoints(proximalSetpoint, distalSetpoint);
        });
    }

    /**
     * Interpolates along a supplied path, ending on the last element in the list

     * @param points the path to follow
     * @param wristAngle the final wrist angle
     * @return a {@link Command}
     */
    public Command interpolateTo(List<Pose2d> points, Angle wristAngle, boolean configuration1) {
        return new Interpolate<Pose2d>(points, 1.0, (pose) -> {
            Angle[] jonitAngles = calculateJointAngles(pose, configuration1);
            io.moveJoints(jonitAngles[0], jonitAngles[1]);
            io.moveWrist(wristAngle);
        }, Interpolate.pose2dInterpolator, system);
    }

    /**
     * Returns the arm to it's home / stowed position
     *
     * @return a {@link Command}
     */
    public Command home() {
        return run(() -> {
            io.moveJoints(Rotations.of(0.25), Rotations.of(0.75));
            io.moveWrist(Rotations.of(0));
        });
    }

    /**
     * Uses 2-axis inverse kinematics to calculate the joint angles
     *
     * @param setpoint the arm setpoint
     * @return the proximal and distal angle setpoints
     */

    private static Angle[] calculateJointAngles(Pose2d setpoint, boolean configuration1) {
        double x = setpoint.getMeasureX().in(Meters);
        double y = setpoint.getMeasureY().in(Meters) - ProximalJoint.GROUND_TO_AXIS.in(Meters);

        double r = Math.sqrt(x * x + y * y);

        double proximalLength = ProximalJoint.LENGTH.in(Meters);
        double distalLength = DistalJoint.LENGTH.in(Meters);

        double cosTheta2Relative = Math.max(
            -1, Math.min(
                1, (r * r - proximalLength * proximalLength - distalLength * distalLength) /
                    (2 * proximalLength * distalLength)
            )
        );

        // Two possible relative angles (elbow up and elbow down)
        double theta2RelativeUp = Math.acos(cosTheta2Relative);
        double theta2RelativeDown = -Math.acos(cosTheta2Relative);

        // Calculate theta1 for both configurations
        double theta1Up = Math.atan2(y, x) - Math.atan2(
            distalLength * Math.sin(theta2RelativeUp),
            proximalLength + distalLength * Math.cos(theta2RelativeUp)
        );

        double theta1Down = Math.atan2(y, x) - Math.atan2(
            distalLength * Math.sin(theta2RelativeDown),
            proximalLength + distalLength * Math.cos(theta2RelativeDown)
        );

        // Normalize angles to [0, 2π)
        theta1Up = ((theta1Up % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
        theta1Down = ((theta1Down % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);

        // Choose configuration based on parameter
        double theta1, theta2Relative;
        if (configuration1) {
            theta1 = theta1Up;
            theta2Relative = theta2RelativeUp;
        } else {
            theta1 = theta1Down;
            theta2Relative = theta2RelativeDown;
        }

        // Calculate theta2 (absolute angle of distal)
        double theta2 = theta1 + theta2Relative;
        theta2 = ((theta2 % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);

        SmartDashboard.putNumber("Calculated theta1", Radians.of(theta1).in(Rotations));
        SmartDashboard.putNumber("Calculated theta2", Radians.of(theta2).in(Rotations));

        return new Angle[] { Radians.of(theta1), Radians.of(theta2) };
    }

    /**
     * Calculates the cartesian pose given the angles of the joints
     *
     * @param jointAngles the angles of the jointsj
     * @return the cartesian pose
     */
    private static Pose2d calculatePose(Angle[] jointAngles) {
        double theta1 = jointAngles[0].in(Radians);
        double theta2 = jointAngles[1].in(Radians);

        double proximalLength = ProximalJoint.LENGTH.in(Meters);
        double distalLength = DistalJoint.LENGTH.in(Meters);

        double x = proximalLength * Math.cos(theta1) + distalLength * Math.cos(theta2);
        double y = proximalLength * Math.sin(theta1) + distalLength * Math.sin(theta2) + ProximalJoint.GROUND_TO_AXIS.in(Meters);

        return new Pose2d(Meters.of(x), Meters.of(y), Rotation2d.kZero);
    }

    public void resetToStartingAngle() {
        io.resetToStartingAngle();
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the L4 position
     *
     * @return the Trigger object
     */
    public Trigger atL4() {
        return atSetpoint(Positions.L4, Positions.L4_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the L3 position
     *
     * @return the Trigger object
     */
    public Trigger atL3() {
        return atSetpoint(Positions.L3, Positions.L3_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the L2 position
     *
     * @return the Trigger object
     */
    public Trigger atL2() {
        return atSetpoint(Positions.L2, Positions.L2_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the L1 position
     *
     * @return the Trigger object
     */
    public Trigger atL1() {
        return atSetpoint(Positions.L1, Positions.L1_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the station position
     *
     * @return the Trigger object
     */
    public Trigger atStation() {
        return atSetpoint(Positions.STATION, Positions.STATION_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the ground intake position
     *
     * @return the Trigger object
     */
    public Trigger atGroundIntake() {
        return atSetpoint(Positions.GROUND_INTAKE, Positions.GROUND_INTAKE_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the l3 algae position
     *
     * @return the Trigger object
     */
    public Trigger atL3Algae() {
        return atSetpoint(Positions.L3, Positions.L3_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the L2 algae position
     *
     * @return the Trigger object
     */
    public Trigger atL2Algae() {
        return atSetpoint(Positions.L2, Positions.L2_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the barge position
     *
     * @return the Trigger object
     */
    public Trigger atBarge() {
        return atSetpoint(Positions.BARGE, Positions.BARGE_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the processor position
     *
     * @return the Trigger object
     */
    public Trigger atProcessor() {
        return atSetpoint(Positions.PROCESSOR, Positions.PROCESSOR_WRIST_ANGLE);
    }

    /**
     * Creates a {@link Trigger} object that is activated if the arm is at the home position
     *
     * @return the Trigger object
     */
    public Trigger atHome() {
        return atSetpoint(new Angle[] { Rotations.of(0.25), Rotations.of(0.0) }, Rotations.of(0.0));
    }

    /**
     * Returns a {@link Trigger} that becomes active if all joints of the arm are within a preset tolerance of the supplied setpoint
     *
     * @param setpoint the queried setpoint
     * @param wristAngle the queried wrist angle
     * @return a {@link Trigger}
     */
    private Trigger atSetpoint(Pose2d setpoint, Angle wristAngle) {
        // TODO: handle configuration1 here
        Angle[] angles = calculateJointAngles(setpoint, false);

        Angle[] currentAngles = io.getJointAngles();
        Angle currentWristAngle = io.getWristAngle();

        double positionTolerance = Positions.POSITION_TOLERANCE.in(Degrees);
        return new Trigger(
            () -> currentAngles[0].minus(angles[0]).abs(Degrees) < positionTolerance &&
                currentAngles[1].minus(angles[1]).abs(Degrees) < positionTolerance &&
                currentWristAngle.minus(wristAngle).abs(Degree) < positionTolerance
        );
    }

    /**
     * Returns a {@link Trigger} that becomes active if all joints of the arm are within a preset tolerance of the supplied setpoint
     *
     * @param setpoint the queried setpoint
     * @param wristAngle the queried wrist angle
     * @return a {@link Trigger}
     */
    private Trigger atSetpoint(Angle[] setpoint, Angle wristAngle) {
        Angle[] currentAngles = io.getJointAngles();
        Angle currentWristAngle = io.getWristAngle();

        double positionTolerance = Positions.POSITION_TOLERANCE.in(Degrees);
        return new Trigger(
            () -> currentAngles[0].minus(setpoint[0]).abs(Degrees) < positionTolerance &&
                currentAngles[1].minus(setpoint[1]).abs(Degrees) < positionTolerance &&
                currentWristAngle.minus(wristAngle).abs(Degree) < positionTolerance
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        Angle[] angles = io.getJointAngles();
        Pose2d calculatedPose = calculatePose(angles);

        SmartDashboard.putNumber("Arm Proximal Angle", angles[0].in(Rotations));
        SmartDashboard.putNumber("Arm Distal Angle", angles[1].in(Rotations));

        SmartDashboard.putNumber("Arm Calculated X", calculatedPose.getX());
        SmartDashboard.putNumber("Arm Calculated Y", calculatedPose.getY());

        SmartDashboard.putNumber("Wrist Angle", io.getWristAngle().in(Degrees));
    }

    /**
     * Returns the singleton instance of the {@link Arm} subystem

     * @return the singleton Arm instance
     */
    public static Arm system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Arm(new ArmIO.Sim());
            else
                system = new Arm(new ArmIO.Real());
        }
        return system;
    }
}