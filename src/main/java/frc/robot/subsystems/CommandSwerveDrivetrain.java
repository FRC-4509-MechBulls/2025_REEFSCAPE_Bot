package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private boolean hasAppliedOperatorPerspective = false;

    private double currentOutputY = 0;
    private double currentOutputX = 0;
    private double rampUp = 0.05;
    private double noInput = 0.25;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

      AutoBuilder.configureHolonomic(
            this.getPose(), // Robot pose supplier
            this.m_odometry.resetPosition(this.m_pigeon2.getRotation2d(), this.m_modulePositions, new Pose2d()), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    ); 
    }
    public Command drive(double leftJoystickY, double leftJoystickX, double rightJoystickX){
        
        if(Math.abs(leftJoystickY - currentOutputY) > 0) {
            if(leftJoystickY == 0) {
                if(currentOutputY > 0) {
                    currentOutputY -= noInput;
                }
                if(currentOutputY < 0) {
                    currentOutputY += noInput;
                }
            }
            else {
                if(leftJoystickY > currentOutputY) {
                    currentOutputY += rampUp;
                    if(currentOutputY > leftJoystickY) {
                        currentOutputY = leftJoystickY;
                    }
                }
                else if(leftJoystickY < currentOutputY) {
                    currentOutputY -= rampUp;
                    if(currentOutputY < leftJoystickY) {
                    currentOutputY = leftJoystickY;
                }
            }
            }
        }

        if(Math.abs(leftJoystickX - currentOutputX) > 0) {
            if(leftJoystickX == 0) {
                if(currentOutputX > 0) {
                    currentOutputX -= noInput;
                }
                if(currentOutputX < 0) {
                    currentOutputX += noInput;
                }
            }
            else {
                if(leftJoystickX > currentOutputX) {
                    currentOutputX += rampUp;
                    if(currentOutputX > leftJoystickX) {
                        currentOutputX = leftJoystickX;
                    }
                }
                else if(leftJoystickX < currentOutputX) {
                    currentOutputX -= rampUp;
                    if(currentOutputX < leftJoystickX) {
                    currentOutputX = leftJoystickX;
                }
            }
            }
        }
        
        return applyRequest(() -> drive.withVelocityX(currentOutputY * MaxSpeed) 
                                        .withVelocityY(currentOutputX * MaxSpeed) 
                                        .withRotationalRate(-rightJoystickX * MaxAngularRate) 
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
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

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true; 
            });
        }
    }

    public Pose2d getPose(){
        return m_odometry.getEstimatedPosition();
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return m_kinematics.toChassisSpeeds(this.m_moduleStates);
     //   this.m_moduleStates;
    } 
    public void drive(ChassisSpeeds speeds){
        drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond);
    }

}