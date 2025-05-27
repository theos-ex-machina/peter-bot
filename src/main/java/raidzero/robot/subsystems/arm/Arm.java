package raidzero.robot.subsystems.arm;

import java.util.List;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.Utils;

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
    public Command moveTo(Pose2d setpoint, Angle wristAngle) {
        return run(() -> {
            Angle[] angles = calculateJointAngles(setpoint);

            io.moveJoints(angles[0], angles[1]);
            io.moveWrist(wristAngle.minus(angles[0]));
        });
    }

    /**
     * Interpolates along a supplied path, ending on the last element in the list
    
     * @param points the path to follow
     * @param wristAngle the final wrist angle
     * @return a {@link Command}
     */
    public Command interpolateTo(List<Pose2d> points, Angle wristAngle) {
        return new Interpolate<Pose2d>(points, 1.0, (pose) -> {
            Angle[] jonitAngles = calculateJointAngles(pose);
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
    private static Angle[] calculateJointAngles(Pose2d setpoint) {
        double x = setpoint.getMeasureX().in(Meters);
        double y = setpoint.getMeasureY().in(Meters);

        double r = Math.sqrt(x * x + y * y);

        double proximalLength = ProximalJoint.LENGTH.in(Meters);
        double distalLength = DistalJoint.LENGTH.in(Meters);

        double cosTheta2 = Math.max(
            -1, Math.min(
                1, (r * r - proximalLength * proximalLength - distalLength * distalLength) /
                    (2 * proximalLength * distalLength)
            )
        );
        double theta2 = Math.acos(cosTheta2);

        double theta1 = Math.atan2(x, y) - Math.atan2(
            distalLength * Math.sin(theta2),
            proximalLength + distalLength * Math.cos(theta2)
        );

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

        double x = proximalLength * Math.sin(theta1) + distalLength * Math.sin(theta1 + theta2);
        double y = proximalLength * Math.cos(theta1) + distalLength * Math.cos(theta1 + theta2);

        return new Pose2d(Meters.of(x), Meters.of(y), Rotation2d.kZero);
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
        Angle[] angles = calculateJointAngles(setpoint);

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
        io.updateTelemetry();

        Angle[] angles = io.getJointAngles();
        Pose2d calculatedPose = calculatePose(angles);

        SmartDashboard.putNumber("Arm Proximal Angle", angles[0].in(Degrees));
        SmartDashboard.putNumber("Arm Distal Angle", angles[1].in(Degrees));

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