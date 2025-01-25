// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.StateControllerSub.AlgaeObjective;
import frc.robot.StateControllerSub.ItemType;
import frc.robot.StateControllerSub.Level;
import frc.robot.StateControllerSub.State;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final StateControllerSub stateController = Constants.RobotConstants.stateController;

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

    InstantCommand setProcessorMode = new InstantCommand(()->stateController.setAlgaeObjective(AlgaeObjective.processor));
    InstantCommand setNetMode = new InstantCommand(()->stateController.setAlgaeObjective(AlgaeObjective.net));

    InstantCommand alignToAprilTag = new InstantCommand(()->drivetrain.alignToAprilTag(0, new Pose2d()));


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

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

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        registerNamedCommands();
        createAutos();
    }
    public void registerNamedCommands() {
        NamedCommands.registerCommand("command name", new InstantCommand());

    }
    public void createAutos(){
        autoChooser.setDefaultOption("no auto", null);

        autoChooser.addOption("Auto", new PathPlannerAuto("Auto"));
        autoChooser.addOption("New Auto", new PathPlannerAuto("New Auto"));
        autoChooser.addOption("Straight", new PathPlannerAuto("Straight"));

        SmartDashboard.putData("autoChooser",autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
