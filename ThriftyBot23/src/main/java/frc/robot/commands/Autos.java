// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase defaultAuto(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
      new RunIntake(intakeSubsystem, -1).withTimeout(2),
      new InstantCommand(()-> drivetrainSubsystem.resetGyro(180)),
      new DriveDistance(drivetrainSubsystem, -0.5, FieldConstants.START_TO_CROSS_DISTANCE)
    );
  }

  public static CommandBase middleBalanceAuto(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
      new RunIntake(intakeSubsystem, -1).withTimeout(2),
      new InstantCommand(()-> drivetrainSubsystem.resetGyro(180)),
      new DriveDistance(drivetrainSubsystem, -0.5, FieldConstants.START_TO_DRIVESTATION),
      new AutoBalance(drivetrainSubsystem)
    );
  } 

  public static CommandBase testAuto(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
    return Commands.sequence(
      new DriveForward(drivetrainSubsystem, -0.5).withTimeout(3)
    );
  }




  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
