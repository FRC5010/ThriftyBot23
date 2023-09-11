// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    motor1 = new CANSparkMax(1,MotorType.kBrushless);
    motor2 = new CANSparkMax(2, MotorType.kBrushless);
  }

  public void runMotor(double speed){
    motor1.set(speed);
    motor2.set(-speed);
  }

  public void stopMotor(){
    motor1.set(0);
    motor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
