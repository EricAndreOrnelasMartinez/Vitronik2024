// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpeakerRigthAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Chassis m_chassis;
  private final Shooter m_shooter;
  private final Intake m_Intake;
  private Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpeakerRigthAuto(Chassis m_chassis, Shooter m_shooter, Intake m_Intake) {
      this.m_chassis = m_chassis;
      this.m_shooter= m_shooter;
      this.m_Intake = m_Intake;
      timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.resetEncoders();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(timer.get()<0.1){
        m_shooter.human(0.5, 0.1);
      }else if(timer.get() > 0.1 && timer.get() < 0.7){
        m_shooter.human(0,0);
      }else if(timer.get() > 1.7 && timer.get() < 2.1){
        m_shooter.human(-0.5, -0.5);
      }else if(timer.get() > 2.1 && timer.get() < 4.1){
        m_shooter.prepareShoot(5676);
      }else if(timer.get() > 4.1 && timer.get() < 6.1){
        m_shooter.shoot(1);      
      }else if(timer.get() > 6.1 && timer.get() < 10){
        m_shooter.stop();
        m_chassis.forward(-0.1);
      }else if(timer.get() > 10 && timer.get() < 10.2){
        m_chassis.resetEncoders();
      }else if(timer.get() > 10.2 && timer.get() < 13.1){
        m_chassis.vitronavx(-0.5);
      }else if(timer.get() > 13.1 && timer.get() < 14 ){
        m_chassis.move(0, 0);
        m_chassis.resetEncoders();
      }else if(timer.get() > 14 && timer.get() < 16){
        m_chassis.forward(-1.5 );
        m_shooter.stop();
        m_Intake.moveIntake(1, 0.3, 0);
      }else if(timer.get() > 16){
        m_chassis.move(0, 0);
      }

    /*if(timer.get()<0.1){
      m_shooter.human(0.5, 0.1);
    }else if(timer.get() > 0.1 && timer.get() < 0.7){
      m_shooter.human(0,0);
    }else if(timer.get() > 1.7 && timer.get() < 2.1){
      m_shooter.human(-0.5, -0.5);
    }else if(timer.get() > 2.1 && timer.get() < 4.1){
      m_shooter.prepareShoot(567.6);
    }else if(timer.get() > 4.1 && timer.get() < 6.1){
      m_shooter.shoot(1);      
    }else if(timer.get() > 6.1 && timer.get() < 6.7){
      m_shooter.stop();
    }else if(timer.get() > 6.7 && timer.get() < 9){
      m_chassis.forward(-0.5);
    }else if(timer.get() > 9 && timer.get() < 11){
      m_chassis.vitronavx(1);
    }else if(timer.get() > 11 && timer.get() < 14){
      m_chassis.forward(1);
    }else{
      m_chassis.move(0, 0);
    }*/

    // if(timer.get()<0.1){
    //    m_shooter.human(0.5, 0.1);
    //  }else if(timer.get() > 0.1 && timer.get() < 0.7){
    //    m_shooter.human(0,0);
    //  }else if(timer.get() > 1.7 && timer.get() < 2.1){
    //    m_shooter.human(-0.5, -0.5);
    //  }else if(timer.get() > 2.1 && timer.get() < 4.1){
    //    //m_shooter.prepareShoot(5676);
    //  }else if(timer.get() > 4.1 && timer.get() < 6.1){
    //    //m_shooter.shoot(1);      
    //  }else if(timer.get() > 6.1 && timer.get() < 6.7){
    //    m_shooter.stop();
    //  }else if(timer.get() > 10 && timer.get() < 14){
    //    m_chassis.forward(-2);
    //  }else{
    //    m_chassis.move(0, 0);
    //  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.move(0, 0);
    m_Intake.stop();
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
