// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.management.loading.PrivateClassLoader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private double TurnSpeed;

  /** Creates a new TurnCommand. */
  public TurnCommand(DrivetrainSubsystem drivetrainSubsystem, double TurnSpeed) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.TurnSpeed = TurnSpeed;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.turnMotors(TurnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
