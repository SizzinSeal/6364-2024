// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  SendableChooser<Integer> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.limelight1.init();
    this.addPeriodic(() -> m_robotContainer.pollBeamBreaks(), 0.002);
    // etc.
    SmartDashboard.putData("Autonomous routine", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  /**
   * @brief sets the current limit of the drivetrain. Useful for making auto
   *        consistent on good
   *        batteries and bad batteries
   * 
   * @param currentLimit The current limit, in amps
   */
  private void setDriveCurrentLimit(double currentLimit) {
    var m0Config = new TalonFXConfiguration();
    var m1Config = new TalonFXConfiguration();
    var m2Config = new TalonFXConfiguration();
    var m3Config = new TalonFXConfiguration();
    m_robotContainer.m_drivetrain.getModule(0).getDriveMotor().getConfigurator().refresh(m0Config);
    m_robotContainer.m_drivetrain.getModule(1).getDriveMotor().getConfigurator().refresh(m1Config);
    m_robotContainer.m_drivetrain.getModule(2).getDriveMotor().getConfigurator().refresh(m2Config);
    m_robotContainer.m_drivetrain.getModule(3).getDriveMotor().getConfigurator().refresh(m3Config);
    m0Config.CurrentLimits.StatorCurrentLimit = currentLimit;
    m1Config.CurrentLimits.StatorCurrentLimit = currentLimit;
    m2Config.CurrentLimits.StatorCurrentLimit = currentLimit;
    m3Config.CurrentLimits.StatorCurrentLimit = currentLimit;
    m0Config.CurrentLimits.StatorCurrentLimitEnable = true;
    m1Config.CurrentLimits.StatorCurrentLimitEnable = true;
    m2Config.CurrentLimits.StatorCurrentLimitEnable = true;
    m3Config.CurrentLimits.StatorCurrentLimitEnable = true;
    m_robotContainer.m_drivetrain.getModule(0).getDriveMotor().getConfigurator().apply(m0Config);
    m_robotContainer.m_drivetrain.getModule(1).getDriveMotor().getConfigurator().apply(m1Config);
    m_robotContainer.m_drivetrain.getModule(2).getDriveMotor().getConfigurator().apply(m2Config);
    m_robotContainer.m_drivetrain.getModule(3).getDriveMotor().getConfigurator().apply(m3Config);
  }

  @Override
  public void autonomousInit() {
    // limit motor currents
    setDriveCurrentLimit(30);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CommandScheduler.getInstance().schedule(m_robotContainer.getAutonomousCommand());

    try {
      int autoMode = autoChooser.getSelected();
    } catch (Exception e) {
      System.out.println(e.toString());
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    // disable motor current limits, turn the bull loose
    setDriveCurrentLimit(60);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
