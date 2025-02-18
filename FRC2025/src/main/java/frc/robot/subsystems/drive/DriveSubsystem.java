// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionConsumer;
import frc.robot.util.pathplanner.LocalADStarAK;

/** Add your docs here. */
public class DriveSubsystem extends SubsystemBase implements VisionConsumer {
    public static final SwerveModuleState[] kSwerveModuleStateNone = new SwerveModuleState[] {};

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
      new Alert("Drive", "Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions =
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private Consumer<List<PathPoint>> pathConsumer = path -> {
        currentPath.clear();
        currentPathPoses.clear();
        for(PathPoint point : path) {
            currentPathPoses.add(
                new Pose2d(point.position, Rotation2d.kZero));
            currentPath.add(point);
        }
    };
    private static List<Pose2d> currentPathPoses = new ArrayList<>();
    private static List<PathPoint> currentPath = new ArrayList<>();

    private Rotation2d desiredHeading = new Rotation2d();
        
    public DriveSubsystem(GyroIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO) {
        this.gyroIO = gyroIO;
        this.modules[0] = new SwerveModule(flIO, 0, ModuleConstants.FRONT_LEFT_CONSTANTS);
        this.modules[1] = new SwerveModule(frIO, 1, ModuleConstants.FRONT_RIGHT_CONSTANTS);
        this.modules[2] = new SwerveModule(blIO, 2, ModuleConstants.BACK_LEFT_CONSTANTS);
        this.modules[3] = new SwerveModule(brIO, 3, ModuleConstants.BACK_RIGHT_CONSTANTS);

        correctionPID.setPID(15, 0.0, 0.0);
        correctionPID.setTolerance(0.1);

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        OdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)),
            DriveConstants.PATH_PLANNER_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        LocalADStarAK pathfinder = new LocalADStarAK();
        pathfinder.setPathConsumer(pathConsumer);
        Pathfinding.setPathfinder(pathfinder);
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                Logger.recordOutput(
                    "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
        
        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, null, null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        if(!Constants.minimalLogging) Logger.processInputs("Drive/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (SwerveModule module : modules) {
                module.stop();
            }
            if(!Constants.minimalLogging) Logger.recordOutput("SwerveStates/Setpoints", kSwerveModuleStateNone);
            if(!Constants.minimalLogging) Logger.recordOutput("SwerveStates/Setpoints", kSwerveModuleStateNone);
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for(int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for(int j = 0; j < 4; j++) {
                modulePositions[j] = modules[j].getOdometryPositions()[i];
                moduleDeltas[j] = new SwerveModulePosition(
                    modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters,
                    modulePositions[j].angle);
                lastModulePositions[j] = modulePositions[j];
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    SwerveModulePosition[] tempPositions = new SwerveModulePosition[4];

    PIDController correctionPID = new PIDController(15.0, 0.0, 0.0);

    public void runVelocity(ChassisSpeeds chassisSpeeds) {
        double correction = 0.0;
        if(Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) > 0.1) {
            //correction = correctionPID.calculate(getRotation().getRadians(), desiredHeading.getRadians());
            correction = -0.2;
        }
        correction = MathUtil.clamp(correction, -0.3, 0.3);

        ChassisSpeeds speeds = new ChassisSpeeds(
            chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond);

        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        if(!Constants.minimalLogging) Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        if(!Constants.minimalLogging) Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].apply(setpointStates[i]);
        }

        if(!Constants.minimalLogging) Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

        desiredHeading = desiredHeading.plus(new Rotation2d(-chassisSpeeds.omegaRadiansPerSecond * 0.02));
    }

    public void runModule(Rotation2d angle, double velocity, int moduleNum) {
        modules[moduleNum].apply(new SwerveModuleState(
            velocity,
            angle
        ));
    }

    public void runModule(double output, int moduleNum) {
        modules[moduleNum].runCharacterization(output);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        //return getPose().getRotation();
        return gyroInputs.yaw;
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    }

    public double getMaxAngularSpeedRadPerSec() {
        return 2 * getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS;
    }

    public static Translation2d[] getModuleTranslations() {
        return DriveConstants.MODULE_OFFSETS;
    }

    public List<PathPoint> getCurrentPath() {
        if (currentPath.size() == 0) {
            return null;
        }
        return currentPath;
    }

    public void zeroGyro(Rotation2d rotation) {
        gyroIO.zeroGyro(rotation);
        desiredHeading = Rotation2d.kZero;
    }
}
