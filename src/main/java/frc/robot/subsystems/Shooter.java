// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motor1;
  private VictorSPX motor2;
  private SparkPIDController m_pidController;
  private double setPoint;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;


  

  public Shooter() {
    motor1 = new CANSparkMax(Constants.MOTOR1, MotorType.kBrushless);
    motor2 = new VictorSPX(Constants.MOTOR2);
    motor1.setInverted(true);
    motor1.setOpenLoopRampRate(0.1);
    m_pidController = motor1.getPIDController();
    

    kP = 5e-4; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0.02; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
  }
  public void neo(double speed){
    motor1.set(speed);
  }

  public void prepareShoot(double setPoint){
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }
  public void prepare(double speed){
    motor1.set(speed);
  }
  public void shoot(double speed){
    motor2.set(ControlMode.PercentOutput, speed);
  }
  
  public void human(double speed1, double speed2){
    motor1.set(speed1);
    motor2.set(ControlMode.PercentOutput, speed2);
  }

  public void stop(){
    motor1.set(0);
    motor2.set(ControlMode.PercentOutput, 0);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
