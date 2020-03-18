package frc.robot.humanIO;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commandGroups.StartCollection;
import frc.robot.commandGroups.StartShooterBottom;
import frc.robot.commandGroups.StartShooterInstant;
import frc.robot.commandGroups.StartShooterUpper;
import frc.robot.commandGroups.StartShooterVision;
import frc.robot.commandGroups.StopCollection;
import frc.robot.commandGroups.StopShooterVision;
import frc.robot.commandGroups.StopShooting;
import frc.robot.commands.drivetrain.GoToYaw;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.drivetrain.SetDrivetrainGear;
import frc.robot.commands.drivetrain.StraightDrive;
import frc.robot.commands.pizza.SetPizzaState;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.Pizza.PizzaState;

public class Joysticks {
  /*
   * Defines a button in a gamepad POV for an array of angles
   */
  private class POVButton extends Button {
    Joystick m_joystick;
    int m_deg;

    public POVButton(Joystick joystick, int deg) {
      m_joystick = joystick;
      m_deg = deg;
    }

    public boolean get() {
      if (m_joystick.getPOV() == m_deg) {
        return true;
      }
      return false;
    }
  }

  private Joystick _leftJoystick, _rightJoystick;
  private XboxController _operatorJoystick;
  private boolean _collecting, _inTankDrive;

  public Joysticks () {
    this._leftJoystick = new Joystick(Constants.Joysticks.LeftJoystick.port);
    this._rightJoystick = new Joystick(Constants.Joysticks.RightJoystick.port);
    this._operatorJoystick = new XboxController(Constants.Joysticks.OperatorJoystick.port);
    this._inTankDrive = true;
    this._collecting = false;
  }

  public boolean isInTankDrive () {
    return this._inTankDrive;
  }

  public void setIsInTankDrive (boolean isInTankDrive) {
    this._inTankDrive = isInTankDrive;
  }

  public double getLeftY() {
    return -this._leftJoystick.getY();
  }

  public double getRightY() {
    return -this._rightJoystick.getY();
  }

  public double getLeftXboxY() {
    return this._operatorJoystick.getY(Hand.kLeft);
  }

  public void startRumble() {
    this._operatorJoystick.setRumble(RumbleType.kLeftRumble, Constants.Joysticks.OperatorJoystick.Rumble.on);
    this._operatorJoystick.setRumble(RumbleType.kRightRumble, Constants.Joysticks.OperatorJoystick.Rumble.on);
  }

  public void stopRumble() {
    this._operatorJoystick.setRumble(RumbleType.kLeftRumble, Constants.Joysticks.OperatorJoystick.Rumble.off);
    this._operatorJoystick.setRumble(RumbleType.kRightRumble, Constants.Joysticks.OperatorJoystick.Rumble.off);
  }

  public double getRightXboxY() {
    return this._operatorJoystick.getY(Hand.kRight);
  }

  public double getLeftXboxTrigger() {
    return this._operatorJoystick.getTriggerAxis(Hand.kLeft);
  }

  public double getRightXboxTrigger() {
    return this._operatorJoystick.getTriggerAxis(Hand.kRight);
  }

  public boolean leftXboxTriggerPressed() {
    return getLeftXboxTrigger() > Constants.Joysticks.OperatorJoystick.triggerThreshold;
  }

  public boolean rightXboxTriggerPressed() {
    return getRightXboxTrigger() > Constants.Joysticks.OperatorJoystick.triggerThreshold;
  }

  public void initButtons() {
    //// Right joystick ////
    JoystickButton straightDrive = new JoystickButton(this._rightJoystick, Constants.Joysticks.RightJoystick.Buttons.straightDrive);
    JoystickButton goToYawButton = new JoystickButton(this._rightJoystick, Constants.Joysticks.RightJoystick.Buttons.goToYaw);

    straightDrive.whenHeld(new StraightDrive());
    goToYawButton.toggleWhenPressed(new GoToYaw());

    //// Left joystick ////
    JoystickButton gearShift = new JoystickButton(this._leftJoystick, Constants.Joysticks.LeftJoystick.Buttons.gearShift);

    gearShift.whenPressed(new InstantCommand(new Runnable(){
      @Override
      public void run() {
        if (Robot.drivetrain.getCurrentGear() == Gear.HIGH) new SetDrivetrainGear(Gear.LOW).schedule();
        else new SetDrivetrainGear(Gear.HIGH).schedule();
      }
    }));

    //// Operator joystick ////
    Trigger upperShoot = new Trigger(this::rightXboxTriggerPressed);
    Trigger upperShootInstant = new Trigger(this::leftXboxTriggerPressed);
    JoystickButton bottomShoot = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.bottomShoot);
    JoystickButton visionShoot = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.visionShoot);
    
    JoystickButton collection = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.collection);
    
    JoystickButton pizzaClockwise = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.pizzaClockwise);
    JoystickButton pizzaCounterClockwise = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.pizzaCounterClockwise);
    
    JoystickButton climb = new JoystickButton(this._operatorJoystick, Constants.Joysticks.OperatorJoystick.Buttons.climb);
    
    // Shooter
    upperShoot.whenActive(new StartShooterUpper());
    upperShoot.whenInactive(new StopShooting());

    upperShootInstant.whenActive(new StartShooterInstant());
    upperShootInstant.whenInactive(new StopShooting());

    bottomShoot.whenPressed(new StartShooterBottom());
    bottomShoot.whenReleased(new StopShooting());

    visionShoot.whenPressed(new StartShooterVision());
    visionShoot.whenReleased(new StopShooterVision());

    // Intake
    collection.whenPressed(new InstantCommand(() -> {
      if (this._collecting) new StopCollection().schedule();
      else new StartCollection().schedule();
      this._collecting = !this._collecting;
    }));

    // Pizza
    pizzaClockwise.whenPressed(new SetPizzaState(PizzaState.CLOCKWISE));
    pizzaClockwise.whenReleased(new SetPizzaState(PizzaState.NONE));

    pizzaCounterClockwise.whenPressed(new SetPizzaState(PizzaState.COUNTERCLOCKWISE));
    pizzaCounterClockwise.whenReleased(new SetPizzaState(PizzaState.NONE));

    // Climber
    climb.whenHeld(new Climb());
  }
}