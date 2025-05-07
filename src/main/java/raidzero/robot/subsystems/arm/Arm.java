package raidzero.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.List;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import raidzero.robot.Constants.Arm.DistalJoint;
import raidzero.robot.Constants.Arm.ProximalJoint;

public class Arm extends SubsystemBase {
    private Pose2d currentPose;

    private static Arm system;
    private ArmIO io;

    private Arm(ArmIO io) {
        this.io = io;

        this.currentPose = new Pose2d(Meters.of(0), Meters.of(0), Rotation2d.kZero);
    }

    /**
     * Moves the arm to the given setpoint with the given wrist angle
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

            currentPose = setpoint;
        });
    }

    public Command interpolateTo(List<Pose2d> points, Angle wristAngle) {
        class FollowPathCommand extends Command {
            private final List<Pose2d> path;
            private final double segmentTime; // seconds to spend between each waypoint
            private final Timer timer = new Timer();

            private int currentSegment = 0;
            private double segmentStartTime = 0.0;

            public FollowPathCommand(List<Pose2d> path, double segmentTimeSeconds) {
                this.path = path;
                this.segmentTime = segmentTimeSeconds;

                super.addRequirements(system);
            }

            @Override
            public void initialize() {
                timer.reset();
                timer.start();
                currentSegment = 0;
                segmentStartTime = 0.0;
            }

            @Override
            public void execute() {
                if (currentSegment >= path.size() - 1)
                    return;

                Pose2d start = path.get(currentSegment);
                Pose2d end = path.get(currentSegment + 1);

                double elapsed = timer.get() - segmentStartTime;
                double t = Math.min(elapsed / segmentTime, 1.0);

                double x = lerp(start.getX(), end.getX(), t);
                double y = lerp(start.getY(), end.getY(), t);

                Pose2d targetPose = new Pose2d(Meters.of(x), Meters.of(y), Rotation2d.kZero);
                Angle[] jointAngles = calculateJointAngles(targetPose);
                io.moveJoints(jointAngles[0], jointAngles[1]);
                io.moveWrist(wristAngle);

                if (t >= 1.0) {
                    currentSegment++;
                    segmentStartTime = timer.get();
                }
            }

            @Override
            public boolean isFinished() {
                return currentSegment >= path.size() - 1;
            }

            @Override
            public void end(boolean interrupted) {
                timer.stop();
                Pose2d finalPose = path.get(path.size() - 1);

                Angle[] jointAngles = calculateJointAngles(finalPose);
                io.moveJoints(jointAngles[0], jointAngles[1]);
                io.moveWrist(wristAngle);
            }

            private double lerp(double a, double b, double t) {
                return a + (b - a) * t;
            }
        }

        return new FollowPathCommand(points, 1.0);
    }

    public Command home() {
        return run(() -> {
            io.moveJoints(Rotations.of(0.25), Rotations.of(0.75));

            if (Intake.system().hasCoral().getAsBoolean()) {
                io.moveWrist(Rotations.of(0));
            } else {
                io.moveWrist(Rotations.of(0.25));
            }
        });
    }

    /**
     * Uses 2-axis inverse kinematics to calculate the joint angles 
     * 
     * @param setpoint the x, y, wrist rotation setpoint
     * @return the proximal and distal angle setpoints in percentage of rotations
     */
    private Angle[] calculateJointAngles(Pose2d setpoint) {
        double x = setpoint.getMeasureX().in(Meters);
        double y = setpoint.getMeasureY().in(Meters);

        double r = Math.sqrt(x * x + y * y); // Distance from origin to target

        double cosTheta2 = Math.max(
            -1, Math.min(
                1, (r * r - ProximalJoint.LENGTH * ProximalJoint.LENGTH - DistalJoint.LENGTH * DistalJoint.LENGTH) /
                    (2 * ProximalJoint.LENGTH * DistalJoint.LENGTH)
            )
        );
        double theta2 = Math.acos(cosTheta2);

        double theta1 = Math.atan2(x, y) - Math.atan2(
            DistalJoint.LENGTH * Math.sin(theta2),
            ProximalJoint.LENGTH + DistalJoint.LENGTH * Math.cos(theta2)
        );

        return new Angle[] { Radians.of(theta1), Radians.of(theta2) };
    }

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