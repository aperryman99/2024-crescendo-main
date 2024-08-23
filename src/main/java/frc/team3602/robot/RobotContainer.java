/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;


import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import monologue.Logged;
import monologue.Annotations.Log;

public class RobotContainer implements Logged {
  // private final PowerDistribution powerDistribution = new PowerDistribution(1,
  // ModuleType.kRev);

  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  public final CommandXboxController guitarController = new CommandXboxController(kGuitarController);
  // Subsystems
  private final DrivetrainSubsystem driveSubsys = new DrivetrainSubsystem(
      kDrivetrainConstants,
      xboxController,
      kFrontLeftModuleConstants,
      kFrontRightModuleConstants,
      kBackLeftModuleConstants,
      kBackRightModuleConstants);

  @Log
  public double targetDistance;

  public final Vision vision = new Vision();
 

  // Operator interfaces
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  private double _kMaxSpeed = kMaxSpeed, _kMaxAngularRate = kMaxAngularRate;

  // Autonomous
  // private final Telemetry logger = new Telemetry(_kMaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    //NamedCommands.registerCommand("autonClockwiseShootCmd", superstructure.autonClockwiseShootCmd());


    // NamedCommands.registerCommand("intakeCmd", superstructure.intakeCmd());

    // NamedCommands.registerCommand("oneNoteMiddle", superstructure.oneNoteMiddle());
    // NamedCommands.registerCommand("oneStartNoteMiddleAmpSide", superstructure.oneStartNoteMiddleAmpSide());
    // NamedCommands.registerCommand("oneLeftMoveShort", superstructure.oneLeftMoveShort());
    // NamedCommands.registerCommand("twoNoteMiddleAmpSide", superstructure.twoNoteMiddleAmpSide());
    // NamedCommands.registerCommand("twoNoteMiddle", superstructure.twoNoteMiddle());
    // NamedCommands.registerCommand("oneNoteLeftAmpSideStart", superstructure.oneNoteLeftAmpSideStart());
    // NamedCommands.registerCommand("twoNoteMiddleEnd", superstructure.twoNoteMiddleEnd());
    // NamedCommands.registerCommand("twoNoteLeftStart", superstructure.twoNoteLeftStart());
    // NamedCommands.registerCommand("twoNoteMiddleAmpSideEnd", superstructure.twoNoteMiddleAmpSideEnd());
    // NamedCommands.registerCommand("oneNoteLeftFirst", superstructure.oneNoteLeftFirst());
    // NamedCommands.registerCommand("twoNoteMoveAmpSideShoot", superstructure.twoNoteMoveAmpSideShoot());
    // NamedCommands.registerCommand("twoNoteRightStart", superstructure.twoNoteRightStart());
    // NamedCommands.registerCommand("twoNoteRightEnd", superstructure.twoNoteRightStart());
    // NamedCommands.registerCommand("oneNoteMoveRightFirst", superstructure.oneNoteMoveRightFirst());
    // NamedCommands.registerCommand("threeNoteMiddleAmpSide", superstructure.threeNoteMiddleAmpSide());
    // NamedCommands.registerCommand("threeFirstNoteMiddleAmpSide", superstructure.threeFirstNoteMiddleAmpSide());
    // // NamedCommands.registerCommand("oneNoteRight", superstructure.oneNoteRight());

    // NamedCommands.registerCommand("oneNoteTwistFirst", superstructure.oneNoteLeftFirst());
    // NamedCommands.registerCommand("twoNoteTwistStart", superstructure.oneNoteLeftFirst());
    // NamedCommands.registerCommand("twoNoteTwistEnd", superstructure.oneNoteLeftFirst());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Drive Polarity", polarityChooser);

    polarityChooser.setDefaultOption("Default", 1.0);
    polarityChooser.addOption("Positive", 1.0);
    polarityChooser.addOption("Negative", -1.0);

    // driveSubsys.registerTelemetry(logger::telemeterize);

    configDefaultCommands();
    configButtonBindings();
  }

  private void configDefaultCommands() {
    driveSubsys
        .setDefaultCommand(driveSubsys.applyRequest(
            () -> driveSubsys.fieldCentricDrive
                .withVelocityX(polarityChooser.getSelected() * xboxController.getLeftY() *
                    _kMaxSpeed)
                .withVelocityY(polarityChooser.getSelected() * xboxController.getLeftX() *
                    _kMaxSpeed)
                .withRotationalRate(-xboxController.getRightX() *
                    _kMaxAngularRate)));

  }

  private void configButtonBindings() {
    // xboxController.leftTrigger(0.5).toggleOnTrue(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed * 0.5;
    // })).toggleOnFalse(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed;
    // }));

    // Xbox controls


    // Triggers
    // new
    // Trigger(intakeSubsys::getColorSensor).onTrue(ledSubsys.setGreen()).onFalse(ledSubsys.setAlliance());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
