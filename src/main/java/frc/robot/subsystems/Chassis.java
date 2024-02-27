// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax mL1;
  private CANSparkMax mL2;
  private CANSparkMax mR1;
  private CANSparkMax mR2;
  private double kp=0.1, ki=0.1, kd=0, kizone=0.025, kff= 0.000156, min=246, max=-246;
  private PIDController pidController;
  private SparkPIDController pid;
  private AHRS giro;

  private Encoder encoderL;
  private Encoder encoderR;

  public Chassis() {
    giro = new AHRS(SPI.Port.kMXP);
    pidController = new PIDController(kp, ki, kd);
    pidController.setIZone(kizone); 
    encoderL = new Encoder(8, 9);
    encoderR = new Encoder(7, 6);
    encoderR.setReverseDirection(false);
    encoderL.setReverseDirection(true);

    mL1 = new CANSparkMax(Constants.MOTORIDL1, MotorType.kBrushless);
    mL2 = new CANSparkMax(Constants.MOTORIDL2, MotorType.kBrushless);
    mR1 = new CANSparkMax(Constants.MOTORIDR1, MotorType.kBrushless);
    mR2 = new CANSparkMax(Constants.MOTORIDR2, MotorType.kBrushless);

    mL1.setInverted(true);
    mL2.setInverted(true);
    mR1.setInverted(false);
    mR2.setInverted(false);

    mL1.setOpenLoopRampRate(0.1);
    mL2.setOpenLoopRampRate(0.1);
    mR1.setOpenLoopRampRate(0.1);
    mR2.setOpenLoopRampRate(0.1);

    mL1.setIdleMode(IdleMode.kBrake);
    mL2.setIdleMode(IdleMode.kBrake);
    mR1.setIdleMode(IdleMode.kBrake);
    mR2.setIdleMode(IdleMode.kBrake);
    
    pid = mL1.getPIDController();   
    pid.setP(kp);
    pid.setD(kd);
    pid.setI(ki);
    pid.setFF(kff);
    pid.setIZone(kizone);
    pid.setOutputRange(min, max);
    SmartDashboard.putNumber("P Gain ", kp);
    SmartDashboard.putNumber("I Gain " , ki );
    SmartDashboard.putNumber("D Gain ", kd);
    SmartDashboard.putNumber("I Zone ", kizone);
    SmartDashboard.putNumber("Feed Forward ", kff);
    SmartDashboard.putNumber("Min Output ", min);
    SmartDashboard.putNumber("Max Output ", max);
  }

  public void move(double x, double y){
    double speedL = (y+(x*0.8));
    double speedR = (y-(x*0.8));
    if((Math.abs(x)<0.1)&&(Math.abs(y)<0.1)){
      speedL=0;
      speedR=0;
    }
    mL1.set(speedL*0.6);
    mL2.set(speedL*0.6);
    mR1.set(speedR*0.6);
    mR2.set(speedR*0.6);
  }

  public void turn(double angle){
    double error = 15;  //-100                        -80
    double absoluteAngle=(giro.getAngle()%360);
    if(angle>((absoluteAngle+180)%360)){
      if((angle>((absoluteAngle-error)%360))&&(angle<((absoluteAngle+error%360)))){
        mL1.set(0);
        mL2.set(0);
        mR1.set(0);
        mR2.set(0);
        System.out.println("angle= "+giro.getAngle());
        System.out.println("ALTO");
        System.out.println("R");
      }else{
        mL1.set(-0.2);
        mL2.set(-0.2);
        mR1.set(0.2);
        mR2.set(0.2);
        System.out.println("angle= "+giro.getAngle());
        System.out.println("ESTOY GIRANDO R");
      }
    }else{
      if((angle>((absoluteAngle-error)%360))&&(angle<((absoluteAngle+error%360))))
      {
        mL1.set(0);
        mL2.set(0);
        mR1.set(0);
        mR2.set(0);
        System.out.println("angle= "+giro.getAngle());
        System.out.println("ALTO");
        System.out.println("L");
      }else{
        mL1.set(0.2);
        mL2.set(0.2);
        mR1.set(-0.2);
        mR2.set(-0.2);
        System.out.println("angle= "+giro.getAngle());
        System.out.println("ESTOY GIRANDO L");
      }
    }
  }

  public void forward(double distance){
    double rotations = (distance*Constants.kPulse)/(Constants.d*Math.PI);
    System.out.println(rotations);
    if((encoderL.getDistance() < rotations)&&(encoderR.getDistance() < rotations)){
      mL1.set(0.2);
      mL2.set(0.2);
      mR1.set(0.2);
      mR2.set(0.2);
    }else{
      mL1.set(0);
      mL2.set(0);
      mR1.set(0);
      mR2.set(0);
    }
    System.out.println(encoderL.getDistance());
  }
  public void forwardPID(double distance){
    double rotations = (distance*Constants.kPulse)/(Constants.d*Math.PI);
    double p = SmartDashboard.getNumber("P Gain " , 0);
    double i = SmartDashboard.getNumber("I Gain ", 0);
    double d = SmartDashboard.getNumber("D Gain ", 0);
    double iz = SmartDashboard.getNumber("I Zone " , 0);
    double ff = SmartDashboard.getNumber("Feed Forward ", 0);
    double max = SmartDashboard.getNumber("Max Output ", 0);
    double min = SmartDashboard.getNumber("Min Output ", 0);

    if((p != pid.getP())) { pid.setP(p);}
    if((i != pid.getI())) { pid.setI(i);}
    if((d != pid.getD())) { pid.setD(d);}
    if((iz != pid.getIZone())) { pid.setIZone(iz);}
    if((ff != pid.getFF())) { pid.setFF(ff);}
    if((max != pid.getOutputMax()) || (min != pid.getOutputMin())) { 
      pid.setOutputRange(min, max);
    }

    pid.setReference(rotations, ControlType.kPosition);
    //pidController.
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
