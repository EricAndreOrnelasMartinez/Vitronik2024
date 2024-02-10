// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax mL1;
  private CANSparkMax mL2;
  private CANSparkMax mR1;
  private CANSparkMax mR2;
  private AHRS giro;

  public Chassis() {
    mL1 = new CANSparkMax(Constants.MOTORIDL1, MotorType.kBrushless);
    mL2 = new CANSparkMax(Constants.MOTORIDL2, MotorType.kBrushless);
    mR1 = new CANSparkMax(Constants.MOTORIDR1, MotorType.kBrushless);
    mR2 = new CANSparkMax(Constants.MOTORIDR2, MotorType.kBrushless);
    mL1.setInverted(true);
    mL2.setInverted(true);
    mR1.setInverted(false);
    mR2.setInverted(false);
    giro = new AHRS(SPI.Port.kMXP);
  }

  public void move(double x, double y){
    double speedL = (y+(x*0.8));
    double speedR = (y-(x*0.8));
    if((Math.abs(x)<0.1)&&(Math.abs(y)<0.1)){
      speedL=0;
      speedR=0;
    }
    mL1.set(speedL*0.3);
    mL2.set(speedL*0.3);
    mR1.set(speedR*0.3);
    mR2.set(speedR*0.3);
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
