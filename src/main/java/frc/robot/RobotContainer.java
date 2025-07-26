// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.StateControllerSub.AlgaeIntakeSource;
import frc.robot.StateControllerSub.AlgaeObjective;
import frc.robot.StateControllerSub.ItemType;
import frc.robot.StateControllerSub.Level;
import frc.robot.StateControllerSub.State;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.maxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond) * Constants.DriveConstants.maxSpeed; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new  SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final XboxController driverController_HID = driverController.getHID();
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    public final StateControllerSub stateController = new StateControllerSub(visionSubsystem, elevatorSubsystem, climbSubsystem, drivetrain);
    

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    InstantCommand setIntakeMode = new InstantCommand(()->stateController.setRobotState(State.intaking));
    InstantCommand setHoldingMode = new InstantCommand(()->stateController.setRobotState(State.holding));
    InstantCommand setPrePlacingMode = new InstantCommand(()->stateController.setRobotState(State.pre_placing));
    InstantCommand setPlacingMode = new InstantCommand(()->stateController.setRobotState(State.placing));

    InstantCommand setLevel1 = new InstantCommand(()->stateController.setLevel(Level.level1));
    InstantCommand setLevel2 = new InstantCommand(()->stateController.setLevel(Level.level2));
    InstantCommand setLevel3 = new InstantCommand(()->stateController.setLevel(Level.level3));
    InstantCommand setLevel4 = new InstantCommand(()->stateController.setLevel(Level.level4));



//    RepeatCommand climbForward = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(.6)));
//    RepeatCommand climbReverse = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(-.6)));
//    RepeatCommand climbStop = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(0)));

    public RobotContainer() {
        configureBindings();
    }

    public CommandSwerveDrivetrain getDriveTrain(){
        return drivetrain;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically // all negative values
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(driverController_HID.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward), changed to use X
                    .withVelocityY(driverController_HID.getLeftX() * MaxSpeed) // Drive left with negative X (left), changed to use Y
                    .withRotationalRate(driverController_HID.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

          
        driverController.rightBumper().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(driverController_HID.getLeftY() * MaxSpeed) // Changed to use x
                .withVelocityY(driverController_HID.getLeftX() * MaxSpeed) // Changed to use y
                .withRotationalRate(driverController_HID.getRightX() * MaxAngularRate)
        ));

        /* 
        driverController.rightTrigger().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(drivetrain.driveToClosestPose().vxMetersPerSecond) // Changed to use x
                .withVelocityY(drivetrain.driveToClosestPose().vyMetersPerSecond) // Changed to use y
                .withRotationalRate(drivetrain.driveToClosestPose().omegaRadiansPerSecond)
        ));

        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(visionSubsystem.driveToAprilTag().vxMetersPerSecond)
                .withVelocityY(visionSubsystem.driveToAprilTag().vyMetersPerSecond)
                .withRotationalRate(visionSubsystem.driveToAprilTag().omegaRadiansPerSecond)));
        */

        driverController.x().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(visionSubsystem.getClampedYawLeft())
                .withRotationalRate(0)));
        driverController.y().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(visionSubsystem.getClampedYawRight())
                .withRotationalRate(0)));

        
        driverController.leftTrigger().onTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(1)
                .withVelocityY(0)
                .withRotationalRate(0)).withTimeout(0.5)
                .andThen(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(visionSubsystem.getClampedYawLeft())
                .withRotationalRate(0))).withTimeout(2)
                .andThen(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(-1)
                .withVelocityY(0)
                .withRotationalRate(0)).withTimeout(0.7)));

        driverController.rightTrigger().onTrue(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(0)
        .withVelocityY(1)
        .withRotationalRate(0)).withTimeout(0.45));
   
/* 
        driverController.rightTrigger().onTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(1)
                .withVelocityY(0)
                .withRotationalRate(0)).withTimeout(0.5)
                .andThen(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(visionSubsystem.getClampedYawRight())
                .withRotationalRate(0))).withTimeout(2)
                .andThen(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(-1)
                .withVelocityY(0)
                .withRotationalRate(0)).withTimeout(0.7)));
*/

        driverController.povUp().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(-1)
                .withVelocityY(0)
                .withRotationalRate(0)));

        driverController.povDown().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(1)
                .withVelocityY(0)
                .withRotationalRate(0)));

        driverController.povLeft().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(-1)
                .withRotationalRate(0)));
        
        driverController.povRight().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(0)
                .withVelocityY(1)
                .withRotationalRate(0)));
        
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        /* 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/
        driverController.leftBumper().onTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric()));
        
//        driverController.leftTrigger().onTrue(drivetrain.runOnce(()->drivetrain.getPigeon2().reset()));
 /* Climb Controls
        driverController.povUp().whileTrue(climbForward);
        driverController.povDown().whileTrue(climbReverse);
        driverController.povUp().or(driverController.povDown()).negate().whileTrue(climbStop);
        driverController.povUp().and(driverController.povDown()).whileFalse(climbStop);
*/

   //     driverController.rightTrigger().onTrue(new AlignToAprilTag(drivetrain, vision, .25).withTimeout(1));
 //       driverController.leftTrigger().onTrue(new AlignToAprilTag2D(drivetrain, vision, 1).withTimeout(5));
       

        operatorController.a().onTrue(setIntakeMode);
        operatorController.b().onTrue(setHoldingMode);
        operatorController.x().onTrue(setPrePlacingMode);
        operatorController.y().onTrue(setPlacingMode);

        operatorController.povDown().onTrue(setLevel1);
        operatorController.povUp().onTrue(setLevel4);
        operatorController.povLeft().onTrue(setLevel2);
        operatorController.povRight().onTrue(setLevel3);

        operatorController.leftTrigger().onTrue(new InstantCommand(()->elevatorSubsystem.resetNumRotations()));
        operatorController.leftBumper().onTrue(new InstantCommand(()->elevatorSubsystem.adjustNumRotations(-1))); // Moves elevator up
        operatorController.rightBumper().onTrue(new InstantCommand(()->elevatorSubsystem.adjustNumRotations(1))); // Moves elevator down
        operatorController.rightTrigger().onTrue(new InstantCommand(()->stateController.toggleControlMode()));

//        operatorController.rightBumper().onTrue(new InstantCommand(()->stateController.rightBumper())); // Toggles between coral and algae
//        operatorController.leftBumper().onTrue(new InstantCommand(()->stateController.leftBumper())); // Toggles algae intake source
//        operatorController.leftTrigger().onTrue(new InstantCommand(()->stateController.leftTrigger())); // Toggles algae placement target (net and processor)

//        operatorController.leftTrigger().onTrue(new InstantCommand(()->stateController.outputCoral(-.3)));
//        operatorController.rightTrigger().onTrue(new InstantCommand(()->stateController.toggleHoldingAlgae()));
//        operatorController.rightTrigger().onTrue(new IntakeCoral(elevator, Constants.ElevatorConstants.beamBreak).withTimeout(5));

//        operatorController.axisGreaterThan(5, .5).whileTrue(outputCoral);
//        operatorController.axisLessThan(5, -.5).whileTrue(reverseCoral);
//        operatorController.axisGreaterThan(5, -.5).and(operatorController.axisLessThan(5, .5)).and(new BooleanSupplier(!stateController.getRobotState().equals(State.intaking))).whileTrue(stopCoral);

        drivetrain.registerTelemetry(logger::telemeterize);
        
        registerNamedCommands();
        createAutos();
    }

    public CommandXboxController getController(){
        return driverController;
    }
    
    public void registerNamedCommands() {
        NamedCommands.registerCommand("setIntakeMode", setIntakeMode);
        NamedCommands.registerCommand("setHoldingMode", setHoldingMode);
        NamedCommands.registerCommand("setPrePlacingMode", setPrePlacingMode);
        NamedCommands.registerCommand("setPlacingMode", setPlacingMode);

//        NamedCommands.registerCommand("setAlgaeMode", setAlgaeMode);
//        NamedCommands.registerCommand("setCoralMode", setCoralMode);

        NamedCommands.registerCommand("setLevel1", setLevel1);
        NamedCommands.registerCommand("setLevel2", setLevel2);
        NamedCommands.registerCommand("setLevel3", setLevel3);
        NamedCommands.registerCommand("setLevel4", setLevel4);

        NamedCommands.registerCommand("Align", drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.5)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(0)
        .withVelocityY(visionSubsystem.getClampedYawLeft())
        .withRotationalRate(0))).withTimeout(2)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(-1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.7)));

    }
    public void createAutos(){
        autoChooser.setDefaultOption("no auto", null);

        // Trouble Shooting - Basic
//        autoChooser.addOption("PPLeave", new PathPlannerAuto("Leave"));

        autoChooser.addOption("Leave", drivetrain.applyRequest(() ->
        robotCentricDrive.withVelocityX(-1) // Drive forward with negative Y (forward), changed to use X
            .withVelocityY(0) // Drive left with negative X (left), changed to use Y
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        ).withTimeout(1));

        autoChooser.addOption("SimpleScoreEdge", drivetrain.applyRequest(() ->
        robotCentricDrive.withVelocityX(-1) // Drive forward with negative Y (forward), changed to use X
            .withVelocityY(0.5) // Drive left with negative X (left), changed to use Y
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        ).withTimeout(4).andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.5)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(0)
        .withVelocityY(visionSubsystem.getClampedYawLeft())
        .withRotationalRate(0))).withTimeout(2)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(-1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.7)))
        .andThen(new InstantCommand(()->stateController.setLevel(Level.level3)))
        .andThen(new InstantCommand(()->stateController.setRobotState(State.pre_placing)))
        .andThen(new WaitCommand(2))
        .andThen(new InstantCommand(()->stateController.setRobotState(State.placing)))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(()->stateController.stopCoral())));

        autoChooser.addOption("SimpleScoreCenter", drivetrain.applyRequest(() ->
        robotCentricDrive.withVelocityX(-1) // Drive forward with negative Y (forward), changed to use X
            .withVelocityY(0) // Drive left with negative X (left), changed to use Y
            .withRotationalRate(0) // Drive counterclockwise with negative X (left)
        ).withTimeout(3).andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.5)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(0)
        .withVelocityY(visionSubsystem.getClampedYawLeft())
        .withRotationalRate(0))).withTimeout(2)
        .andThen(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(-1)
        .withVelocityY(0)
        .withRotationalRate(0)).withTimeout(0.7)))
        .andThen(new InstantCommand(()->stateController.setLevel(Level.level3)))
        .andThen(new InstantCommand(()->stateController.setRobotState(State.pre_placing)))
        .andThen(new WaitCommand(2))
        .andThen(new InstantCommand(()->stateController.setRobotState(State.placing)))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(()->stateController.stopCoral())));

/* 
        autoChooser.addOption("Center1Score", drivetrain.applyRequest(() ->
        robotCentricDrive.withVelocityX(0) 
            .withVelocityY(MaxSpeed/3) 
            .withRotationalRate(0) 
        ).withTimeout(1).andThen(setLevel3).andThen(setPrePlacingMode).andThen(new WaitCommand(2)).andThen(setPlacingMode));
*/


        // 1 Coral Simple Autos
        autoChooser.addOption("OR-R6-1S", new PathPlannerAuto("OBR-R6-1S"));
        autoChooser.addOption("OR-R1-1S", new PathPlannerAuto("OBR-R1-1S"));
        autoChooser.addOption("OR-R2-1S", new PathPlannerAuto("OBR-R2-1S"));
        autoChooser.addOption("OL-R4-1S", new PathPlannerAuto("OBL-R4-1S"));
        autoChooser.addOption("OL-R3-1S", new PathPlannerAuto("OBL-R3-1S"));
        autoChooser.addOption("OL-R2-1S", new PathPlannerAuto("OBL-R2-1S"));
        autoChooser.addOption("I-R5-1S", new PathPlannerAuto("I-R5-1S"));

/*
        // Coral Cycle (Coral Station 1 and 2)
        autoChooser.addOption("S1-CC", new PathPlannerAuto("S1-CC"));
        autoChooser.addOption("S2-CS1-CC", new PathPlannerAuto("S2-CS1-CC"));
        autoChooser.addOption("S2-CS2-CC", new PathPlannerAuto("S2-CS2-CC"));
        autoChooser.addOption("S3-CC", new PathPlannerAuto("S3-CC"));

        // Algae Proccessor - Coral Cycle
        autoChooser.addOption("S1-AP-CC", new PathPlannerAuto("S1-AP-CC"));
        autoChooser.addOption("S2-AP-CC", new PathPlannerAuto("S2-AP-CC"));
        autoChooser.addOption("S3-AP-CC", new PathPlannerAuto("S3-AP-CC"));

        // Opposite Coral Station - Coral Cycle (Sweep and No Sweep)
        autoChooser.addOption("S1-OCS-CC", new PathPlannerAuto("S1-OCS-CC"));
        autoChooser.addOption("S1-OCS-S-CC", new PathPlannerAuto("S1-OCS-S-CC"));
        autoChooser.addOption("S2-CS1-OCS-CC", new PathPlannerAuto("S2-CS1-OCS-CC"));
        autoChooser.addOption("S2-CS1-OCS-S-CC", new PathPlannerAuto("S2-CS1-OCS-S-CC"));
        autoChooser.addOption("S2-CS2-OCS-CC", new PathPlannerAuto("S2-CS2-OCS-CC"));
        autoChooser.addOption("S2-CS2-OCS-S-CC", new PathPlannerAuto("S2-CS2-OCS-S-CC"));
        autoChooser.addOption("S3-OCS-CC", new PathPlannerAuto("S3-OCS-CC"));
        autoChooser.addOption("S3-OCS-S-CC", new PathPlannerAuto("S3-OCS-S-CC"));

        // Shooting Position - Coral Cycle
        autoChooser.addOption("S1-SP2-AC", new PathPlannerAuto("S1-SP2-AC"));
        autoChooser.addOption("S1-SP1-AC", new PathPlannerAuto("S1-SP1-AC"));
        autoChooser.addOption("S2-SP2-AC", new PathPlannerAuto("S2-SP2-AC"));
        autoChooser.addOption("S2-SP1-AC", new PathPlannerAuto("S2-SP1-AC"));
        autoChooser.addOption("S3-SP2-AC", new PathPlannerAuto("S3-SP2-AC"));
        autoChooser.addOption("S3-SP1-AC", new PathPlannerAuto("S3-SP1-AC"));
        */

        SmartDashboard.putData("autoChooser",autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}