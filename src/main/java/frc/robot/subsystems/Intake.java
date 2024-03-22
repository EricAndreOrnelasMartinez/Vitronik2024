// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax mIntake;
  private VictorSPX motor2;
  
  //private Shooter m_Shooter;


  public static DigitalInput limit;

  public Intake() {
    mIntake = new CANSparkMax(Constants.IDINTAKE, MotorType.kBrushless);
    motor2 = new VictorSPX(Constants.MOTOR2);
    limit = new DigitalInput(5);
  
    mIntake.setInverted(true);
    
  }
  public void moveIntake(double speed, double speed2, double speed3){
    if(!limit.get()){
      mIntake.set(0);
      motor2.set(ControlMode.PercentOutput, 0);
    }else{
      mIntake.set(speed);
      motor2.set(ControlMode.PercentOutput, speed2);
    }
  }
  public void outIntake(double speed){
    mIntake.set(speed);
    motor2.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    mIntake.set(0);
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
