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

import commands.AlignToAprilTag;
import commands.AlignToAprilTag2D;
import commands.IntakeCoral;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.StateControllerSub.AlgaeIntakeSource;
import frc.robot.StateControllerSub.AlgaeObjective;
import frc.robot.StateControllerSub.ItemType;
import frc.robot.StateControllerSub.Level;
import frc.robot.StateControllerSub.State;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.DriveConstants.maxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * .1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final StateControllerSub stateController = new StateControllerSub();
    private final VisionSubsystem vision = stateController.getVisionSubsystem();
    private final ElevatorSubsystem elevator = stateController.getElevator();

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    InstantCommand setIntakeMode = new InstantCommand(()->stateController.setRobotState(State.intaking));
    InstantCommand setHoldingMode = new InstantCommand(()->stateController.setRobotState(State.holding));
    InstantCommand setPrePlacingMode = new InstantCommand(()->stateController.setRobotState(State.pre_placing));
    InstantCommand setPlacingMode = new InstantCommand(()->stateController.setRobotState(State.placing));
    InstantCommand setClimbMode = new InstantCommand(()->stateController.setRobotState(State.climbing));

    InstantCommand setAlgaeMode = new InstantCommand(()->stateController.setItemType(ItemType.algae));
    InstantCommand setCoralMode = new InstantCommand(()->stateController.setItemType(ItemType.coral));

    InstantCommand setLevel1 = new InstantCommand(()->stateController.setLevel(Level.level1));
    InstantCommand setLevel2 = new InstantCommand(()->stateController.setLevel(Level.level2));
    InstantCommand setLevel3 = new InstantCommand(()->stateController.setLevel(Level.level3));
    InstantCommand setLevel4 = new InstantCommand(()->stateController.setLevel(Level.level4));

    InstantCommand setShooterIntakeLevel3 = new InstantCommand(()->stateController.setAlgaeIntakeSource(AlgaeIntakeSource.level3));
    InstantCommand setShooterIntakeLevel2 = new InstantCommand(()->stateController.setAlgaeIntakeSource(AlgaeIntakeSource.level2));

    InstantCommand setProcessorMode = new InstantCommand(()->stateController.setAlgaeObjective(AlgaeObjective.processor));
    InstantCommand setNetMode = new InstantCommand(()->stateController.setAlgaeObjective(AlgaeObjective.net));

    InstantCommand alignToAprilTag = new InstantCommand(()->drivetrain.alignToAprilTag(vision.getPipelineResult(), vision.getAprilTagFieldLayout()));

    RepeatCommand climbForward = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(.6)));
    RepeatCommand climbReverse = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(-.6)));
    RepeatCommand climbStop = new RepeatCommand(new InstantCommand(()->stateController.toggleClimb(0)));

    RunCommand stopCoral = new RunCommand(()->stateController.stopCoral());
    RepeatCommand outputCoral = new RepeatCommand(new InstantCommand(()->stateController.outputCoral()));
    RepeatCommand reverseCoral = new RepeatCommand(new InstantCommand(()->stateController.reverseCoral()));

    RunCommand chassisSpeedDrive = new RunCommand(()->drivetrain.drive(new ChassisSpeeds(driverController.getLeftX() * MaxSpeed, driverController.getLeftY() * MaxSpeed, driverController.getRightX() * MaxAngularRate)));

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(driverController.getLeftX() * MaxSpeed) // Drive forward with negative Y (forward), changed to use X
                    .withVelocityY(-driverController.getLeftY() * MaxSpeed) // Drive left with negative X (left), changed to use Y
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.rightBumper().whileTrue(drivetrain.applyRequest(()->
                robotCentricDrive.withVelocityX(driverController.getLeftX() * MaxSpeed) // Changed to use x
                .withVelocityY(-driverController.getLeftY() * MaxSpeed) // Changed to use y
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
        ));



        
 //       drivetrain.setDefaultCommand(drivetrain.drive(-driverController.getLeftY() * MaxSpeed, -driverController.getLeftX() * MaxSpeed, -driverController.getRightX() * MaxAngularRate));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.povUp().whileTrue(climbForward);
        driverController.povDown().whileTrue(climbReverse);
        driverController.povUp().or(driverController.povDown()).negate().whileTrue(climbStop);

        driverController.povUp().and(driverController.povDown()).whileFalse(climbStop);

   //     driverController.rightTrigger().onTrue(new AlignToAprilTag(drivetrain, vision, .25).withTimeout(1));
        driverController.leftTrigger().onTrue(new AlignToAprilTag2D(drivetrain, vision, 1).withTimeout(5));
        
        driverController.rightTrigger().whileTrue(drivetrain.applyRequest(()->
        robotCentricDrive.withVelocityX(-vision.getYaw()) 
        .withVelocityY(-driverController.getLeftY() * MaxSpeed) 
        .withRotationalRate(driverController.getRightX() * MaxAngularRate)
        ));

        driverController.povLeft().onTrue(new InstantCommand(()->stateController.setAlignmentPoint(17)));
        driverController.povRight().onTrue(new InstantCommand(()->stateController.setAlignmentPoint(-17)));
        driverController.povUp().onTrue(new InstantCommand(()->stateController.setAlignmentPoint(0)));

        operatorController.a().onTrue(setIntakeMode);
        operatorController.b().onTrue(setHoldingMode);
        operatorController.x().onTrue(setPrePlacingMode);
        operatorController.y().onTrue(setPlacingMode);

        operatorController.povDown().onTrue(setLevel1);
        operatorController.povUp().onTrue(setLevel4);
        operatorController.povLeft().onTrue(setLevel2);
        operatorController.povRight().onTrue(setLevel3);

        operatorController.leftTrigger().onTrue(new InstantCommand(()->elevator.resetNumRotations()));
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
        NamedCommands.registerCommand("command name", new InstantCommand());
        NamedCommands.registerCommand("setIntakeMode", setIntakeMode);
        NamedCommands.registerCommand("setHoldingMode", setHoldingMode);
        NamedCommands.registerCommand("setPrePlacingMode", setPrePlacingMode);
        NamedCommands.registerCommand("setPlacingMode", setPlacingMode);
        NamedCommands.registerCommand("setClimbMode", setClimbMode);

        NamedCommands.registerCommand("setAlgaeMode", setAlgaeMode);
        NamedCommands.registerCommand("setCoralMode", setCoralMode);

        NamedCommands.registerCommand("setLevel1", setLevel1);
        NamedCommands.registerCommand("setLevel2", setLevel2);
        NamedCommands.registerCommand("setLevel3", setLevel3);
        NamedCommands.registerCommand("setLevel4", setLevel4);

        NamedCommands.registerCommand("intakeCoral", new IntakeCoral(elevator, Constants.ElevatorConstants.beamBreak));

        NamedCommands.registerCommand("setProcessorMode", setProcessorMode);
        NamedCommands.registerCommand("setNetMode", setNetMode);

        NamedCommands.registerCommand("alignToAprilTag", alignToAprilTag);

    }
    public void createAutos(){
        autoChooser.setDefaultOption("no auto", null);

         
        // Trouble Shooting - Basic
        autoChooser.addOption("Straight", new PathPlannerAuto("Straight"));
        autoChooser.addOption("New Auto", new PathPlannerAuto("New Auto"));
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