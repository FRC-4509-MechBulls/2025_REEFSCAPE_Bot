package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.LegacySwerveControlRequestParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import commands.AlignToPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;




    private double currentOutputY = 0;
    private double currentOutputX = 0;
    private double rampUp = 0.05;
    private double noInput = 0.25;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private Field2d field = new Field2d();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveDrivePoseEstimator poseEstimator;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        SmartDashboard.putData("field",field);
        
        for(int i = 0; i < 4; i++){
            modulePositions[i] = this.getModule(i).getPosition(true);
        }
        poseEstimator = new SwerveDrivePoseEstimator(getKinematics(), this.getPigeon2().getRotation2d(), modulePositions, this.getState().Pose);
        
        
        
        AutoBuilder.configure(
            this::getPose,
            this::resetEstimatedPose,
            this::getRobotRelativeSpeeds,
            this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(5,0,0), 
                new PIDConstants(5,0,0) 
                ),
            Constants.AutoConstants.robotConfig,
            () -> {
                // Boolean that controls when the path will be mirrored
                // Origin remains on blue side, this will flip the path being followed to the red side
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
        SmartDashboard.putNumber("heading", this.getPigeon2().getYaw().getValueAsDouble());
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
    public void resetEstimatedPose(Pose2d newPose){
        poseEstimator.resetPose(newPose);
        this.resetPose(newPose);
    }
    public void drive(ChassisSpeeds chassisSpeeds){

        SwerveModuleState[] moduleStates = getKinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MaxSpeed);
        for(int i = 0; i < 4; i++) {
//            moduleStates[i].speedMetersPerSecond = drivefeedforward.calculate(moduleStates[i].speedMetersPerSecond);
            moduleStates[i].optimize(this.getModule(i).getCurrentState().angle);
//            this.getModule(i).getDriveMotor().set(moduleStates[i].speedMetersPerSecond);
//            this.getModule(i).getSteerMotor().set(steerController.calculate(this.getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble()*360, MathUtil.inputModulus(moduleStates[i].angle.getDegrees(), -180, 180)));
   //         this.getModule(i).getSteerMotor().setControl(positionVoltage.withPosition(moduleStates[i].angle.getRadians()));
}    
        
        this.getModule(0).apply(new ModuleRequest().withState(moduleStates[0]));
        this.getModule(1).apply(new ModuleRequest().withState(moduleStates[1]));
        this.getModule(2).apply(new ModuleRequest().withState(moduleStates[2]));
        this.getModule(3).apply(new ModuleRequest().withState(moduleStates[3]));
    
    }

    public Command driveIsolated(int i, double leftJoystickX) {

        ChassisSpeeds speed = new ChassisSpeeds(0,0, leftJoystickX);
        SwerveModuleState[] moduleState = getKinematics().toSwerveModuleStates(speed);
        
        
        return 
        this.applyRequest(() ->
        drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(leftJoystickX * MaxAngularRate) // Drive counterclockwise with negative X (left)
);

    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            modulePositions[i] = this.getModule(i).getPosition(true);
        }
        return modulePositions;
    }

    @Override
    public void periodic() {
        field.setRobotPose(this.getPose());
        poseEstimator.update(this.getPigeon2().getRotation2d(), getModulePositions());
        SmartDashboard.putNumberArray("pose", new Double[]{
            poseEstimator.getEstimatedPosition().getX(),
            poseEstimator.getEstimatedPosition().getY(),
            poseEstimator.getEstimatedPosition().getRotation().getRadians()
        });

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose(){
        Pose2d originalPose = poseEstimator.getEstimatedPosition();
        Rotation2d adjustedRotation = Rotation2d.fromDegrees(originalPose.getRotation().getDegrees() - 90);

        return new Pose2d(originalPose.getTranslation(), adjustedRotation);
 //       return this.getState().Pose;
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
        
   }
    public Command drive(double leftJoystickY, double leftJoystickX, double rightJoystickX){
            return this.applyRequest(() ->
                      drive.withVelocityX(currentOutputY * MaxSpeed) // Drive forward with negative Y (forward)
                          .withVelocityY(currentOutputX * MaxSpeed) // Drive left with negative X (left)
                          .withRotationalRate(rightJoystickX * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    public Command alignToAprilTag(PhotonPipelineResult result, AprilTagFieldLayout aprilTagLayout) {

        // return an instance of AlignToPose

            int tagID;
            Pose2d targetPoseRelativeToTag = new Pose2d();

            if(result.hasTargets()){
                var target = result.getBestTarget();
                tagID = target.getFiducialId();
                switch(tagID){
                    case 6: targetPoseRelativeToTag = Constants.DriveConstants.tag6Pose;
                        break;
                    case 7: targetPoseRelativeToTag = Constants.DriveConstants.tag7Pose;
                        break;
                    case 8: targetPoseRelativeToTag = Constants.DriveConstants.tag8Pose;
                        break;
                    case 9: targetPoseRelativeToTag = Constants.DriveConstants.tag9Pose;
                        break;
                    case 10: targetPoseRelativeToTag = Constants.DriveConstants.tag10Pose;
                        break;
                    case 11: targetPoseRelativeToTag = Constants.DriveConstants.tag11Pose;
                        break;
                    case 17: targetPoseRelativeToTag = Constants.DriveConstants.tag11Pose;
                        break;
                    case 18: targetPoseRelativeToTag = Constants.DriveConstants.tag10Pose;
                        break;
                    case 19: targetPoseRelativeToTag = Constants.DriveConstants.tag9Pose;
                        break;
                    case 20: targetPoseRelativeToTag = Constants.DriveConstants.tag8Pose;
                        break;
                    case 21: targetPoseRelativeToTag = Constants.DriveConstants.tag7Pose;
                        break;
                    case 22: targetPoseRelativeToTag = Constants.DriveConstants.tag6Pose;
                        break;
                }

                Optional<Pose3d> tagPose3d = aprilTagLayout.getTagPose(target.fiducialId);
                if(tagPose3d.isPresent()){
                    Pose2d tagPose = tagPose3d.get().toPose2d();
                    Pose2d targetPose = tagPose.plus(new Transform2d(targetPoseRelativeToTag.getTranslation(), targetPoseRelativeToTag.getRotation()));
 
                    return new AlignToPose(this, targetPose);
                }
            }
            return new InstantCommand();
    }
}
