package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Utils;
import frc.robot.Constants;
import frc.robot.commands.pizza.SetPizzaState;

/**
 * Pizza subsystem
 */
public class Pizza extends DBugSubsystem {
    private DBugTalon _pizzaMotor;
    private PizzaState _currentState = PizzaState.NONE;

    public Pizza() {
        this._pizzaMotor = (DBugTalon) Utils.getBean("pizza.pizzaMotor");
        
        this._pizzaMotor.setInverted(Constants.Pizza.MotorControllers.PizzaMotor.inverted);

        this._pizzaMotor.configVoltageCompSaturation(Constants.Pizza.MotorControllers.PizzaMotor.voltageCompensation);
        this._pizzaMotor.enableVoltageCompensation(true);
    }

    /**
     * The state of the pizza which spins the ball into the extractor.
     */
    public enum PizzaState {
        CLOCKWISE(Constants.Pizza.PizzaState.CLOCKWISE.percent), //The percent the motor should move at for now its just random numbers until we will have more data about the subsystem.
        CLOCKWISE_BOTTOM(Constants.Pizza.PizzaState.CLOCKWISE_BOTTOM.percent),
        COUNTERCLOCKWISE(Constants.Pizza.PizzaState.COUNTERCLOCKWISE.percent),
        NONE(Constants.Pizza.PizzaState.NONE.percent);

        private double _percent;
        
        PizzaState(double percent) {
            this._percent = percent;
        }

        public double getPercent() {
            return this._percent;
        }
    }

    public void setPizzaState(PizzaState wantedState) {
        this._pizzaMotor.set(ControlMode.PercentOutput, wantedState.getPercent());
        this._currentState = wantedState;
    }

    /**
     * @return The current pizza state.
     */
    public PizzaState getPizzaState() {
        return this._currentState;
    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayTestData() {
        SmartDashboard.putString("Pizza State", this.getPizzaState().toString());
    }

    @Override
    public void displayMatchData() {
        SmartDashboard.putBoolean("Pizza CLOCKWISE", this.getPizzaState() == PizzaState.CLOCKWISE);
        SmartDashboard.putBoolean("Pizza COUNTERCLOCKWISE", this.getPizzaState() == PizzaState.COUNTERCLOCKWISE);
    }

    @Override
    public void displayCommands() {
        SmartDashboard.putData("Pizza CLOCKWISE", new SetPizzaState(PizzaState.CLOCKWISE));
        SmartDashboard.putData("Pizza COUNTERCLOCKWISE", new SetPizzaState(PizzaState.COUNTERCLOCKWISE));
        SmartDashboard.putData("Pizza NONE", new SetPizzaState(PizzaState.NONE));
    }
}