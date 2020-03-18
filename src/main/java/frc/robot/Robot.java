/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoPlan;
import frc.robot.commandGroups.StopShooting;
import frc.robot.commands.drivetrain.SetDrivetrainGear;
import frc.robot.commands.drivetrain.TankDrive;
import frc.robot.commands.general.DisableCompressor;
import frc.robot.commands.general.EnableCompressor;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.humanIO.Joysticks;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PCPoosher;
import frc.robot.subsystems.Pizza;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.PCPoosher.PCPoosherState;
import frc.robot.vision.VisionServer;
import frc.robot.commands.PCPoosher.SetPCPoosherState;
import frc.robot.commands.climb.SetClimberArmJoysticks;
import frc.robot.commands.drivetrain.GoToYaw;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /*
  * Vision Server
  */
  public static VisionServer visionServer;

  /*
   * Subsystems
   */
  public static Drivetrain drivetrain;
  public static Intake intake;
  public static Pizza pizza;
  public static Extractor extractor;
  public static Shooter shooter;
  public static Climber climber;
  public static PCPoosher pcPoosher;

  /**
   * HumanIO
   */
  public static Joysticks joysticks;

  /**
   * Compressor
   */
  public static Compressor compressor;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    /*
     * Subsystems
     */
    Robot.intake = new Intake();
    Robot.pizza = new Pizza();
    Robot.extractor = new Extractor();
    Robot.drivetrain = new Drivetrain();
    Robot.shooter = new Shooter();
    Robot.climber = new Climber();
    Robot.pcPoosher = new PCPoosher();

    /**
     * HumanIO
     */
    Robot.joysticks = new Joysticks();
    joysticks.initButtons();

    
    /**
     * SDB commands
     */
    // Robot.shooter.displayCommands();
    // Robot.pizza.displayCommands();
    // Robot.extractor.displayCommands();
    // Robot.shooter.displayCommands();

    /**
     * Compressor
     */
    Robot.compressor = new Compressor();
    compressor.setClosedLoopControl(true);

    /**
     * Vision Server
     */
    visionServer = new VisionServer("Microsoft LifeCam HD-3000");

    SmartDashboard.putData(new EnableCompressor());
    SmartDashboard.putData(new DisableCompressor());

    SmartDashboard.putData("pc poosher out", new SetPCPoosherState(PCPoosherState.OUT));
    SmartDashboard.putData("pc poosher in", new SetPCPoosherState(PCPoosherState.IN));
  } 

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Robot.drivetrain.displayMatchData();
    Robot.visionServer.displayMatchData();

    //Robot.climber.displayTestData();
    // Robot.extractor.displayTestData();
    
    SmartDashboard.putNumber("visionDistance", Robot.visionServer.getNormalizeDistance());

    // System.out.println(Robot.visionServer.getTargetRectangleHeight());

    //SmartDashboard.putBoolean("in switch", Robot.intake.getInSwitch());
    //SmartDashboard.putBoolean("out switch", Robot.intake.getOutSwitch());
    //SmartDashboard.putNumber("arm percent", Robot.intake.getArmPercent());

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    new EnableCompressor().schedule();
    /*
    if (Constants.Autonomous.autoPlan != AutoPlan.None) {
      if (Robot.drivetrain.getCurrentGear() != Gear.HIGH) new SetDrivetrainGear(Gear.HIGH).schedule();
      Constants.Autonomous.autoPlan.getPlan().schedule();
    }
    */
    new SetPCPoosherState(PCPoosherState.OUT).schedule();
    new SetIntakeState(IntakePosition.IN).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**	
   * This function is called once each time the robot enters teleoperated mode.	
   */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancel(Constants.Autonomous.autoPlan.getPlan());
    new StopShooting().schedule();
    Robot.drivetrain.setDefaultCommand(new TankDrive());
    new EnableCompressor().schedule();
    new SetClimberArmJoysticks().schedule();
    new SetIntakeState(IntakePosition.IN).schedule();
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
