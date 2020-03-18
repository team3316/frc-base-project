package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Utils;
import frc.robot.Constants;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.intake.SetRollersState;
/**
 * Intake
 */
public class Intake extends DBugSubsystem {
    
    private DBugTalon _rollerVictor;
    private DBugTalon _armTalon;

    private IntakePosition _wantedPosition;

    public Intake() {
        this._rollerVictor = (DBugTalon) Utils.getBean("intake.rollervictor");
        this._armTalon = (DBugTalon) Utils.getBean("intake.armTalon");

        this._rollerVictor.configFactoryDefault();
        
        this._rollerVictor.setInverted(Constants.Intake.MotorControllers.RollerVictor.inverted);

        this._rollerVictor.configVoltageCompSaturation(Constants.Intake.rollerVoltageCompensation);
        this._rollerVictor.enableVoltageCompensation(true);
        
        this._armTalon.setInverted(Constants.Intake.MotorControllers.ArmTalon.inverted);

        this._armTalon.configVoltageCompSaturation(Constants.Intake.armVoltageCompensation);
        this._armTalon.enableVoltageCompensation(true);
        
        this._armTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        this._armTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        this._wantedPosition = IntakePosition.DEFAULT;
    }

    /**
     * An enum which represents the state of the intake.
     */
    public enum IntakePosition {
        IN(Constants.Intake.IntakePosition.IN.percent, Constants.Intake.IntakePosition.KEEPIN.percent),
        OUT(Constants.Intake.IntakePosition.OUT.percent, Constants.Intake.IntakePosition.KEEPOUT.percent),
        DEFAULT(Constants.Intake.IntakePosition.DEFAULT.percent, Constants.Intake.IntakePosition.DEFAULT.percent);

        private double _percent, _keepPercent;

        private IntakePosition(double percent, double keepPercent) {
            this._percent = percent;
            this._keepPercent = keepPercent;
        }

        public double getValue() {
            return this._percent;
        }

        public double getKeepValue() {
            return this._keepPercent;
        }
    }

    /**
     * An enum which represents the state of the intake roller.
     */
    public enum RollerState {
        IN(Constants.Intake.Roller.RollerState.IN.voltage),
        OUT(Constants.Intake.Roller.RollerState.OUT.voltage),
        DEFAULT(Constants.Intake.Roller.RollerState.DEFAULT.voltage);

        private double _voltage;

        private RollerState(double voltage) {
            this._voltage = voltage;
        }

        public double getValue() {
            return this._voltage;
        }
    }

    @Override

    /**
     * if one of the sensors is on - set system distance to the sensor's predefined distance
     */
    public void periodic() {
        
    }

    public double getArmPercent() {
        return this._armTalon.getMotorOutputPercent();
    }

    /**
     * Sets roller state
     * 
     * @param state - the RollerState to set.
     */
    public void setRollerState(RollerState state) {
        this._rollerVictor.set(ControlMode.PercentOutput, state.getValue());
    }

    /**
     * Gets current roller state
     * 
     * @return the roller state
     */
    public RollerState getRollerState() {
        double percent = this._rollerVictor.getMotorOutputPercent();
        if (Utils.isInNeighborhood(percent, RollerState.IN.getValue(), Constants.Intake.Roller.tolerance)) {
            return RollerState.IN;
        }

        if (Utils.isInNeighborhood(percent, RollerState.OUT.getValue(), Constants.Intake.Roller.tolerance)) {
            return RollerState.OUT;
        }

        return RollerState.DEFAULT;
    }

    /**
     * Sets intake position
     * 
     * @param _wantedState - the IntakePosition to set.
     */
    public void setPosition(IntakePosition pos) {
        this._armTalon.set(ControlMode.PercentOutput, pos.getValue());
        this._wantedPosition = pos;
    }

    /**
     * Sets intake position to keep values
     * @param _wantedState - the IntakePosition to set.
     */
    public void setPositionKeep(IntakePosition pos) {
        this._armTalon.set(ControlMode.PercentOutput, pos.getKeepValue());
        this._wantedPosition = pos;
    }

    /**
     * Gets the IN switch's value
     * 
     * @return the IN switch's value
     */
    public boolean getInSwitch() {
        return this._armTalon.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Gets the OUT switch's value
     * 
     * @return the IN switch's value
     */
    public boolean getOutSwitch() {
        return this._armTalon.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * Gets current intake position
     * 
     * @return the intake's current position
     */
    public IntakePosition getPosition() {
        if (this.getInSwitch()) {
            return IntakePosition.IN;
        }

        if (this.getOutSwitch()) {
            return IntakePosition.OUT;
        }

        return IntakePosition.DEFAULT;
    }

    /**
     * A function which returns the wanted intake of the position.
     * @return The last input given to the setPosition function.
     */
    public IntakePosition getWantedPosition() {
        return this._wantedPosition;
    }

    @Override
    public void displayTestData() {
        // TODO Auto-generated method stub

    }

    @Override
    public void displayMatchData() {
        SmartDashboard.putBoolean("Intake IN", this.getPosition() == IntakePosition.IN);
        SmartDashboard.putBoolean("Intake OUT", this.getPosition() == IntakePosition.OUT);

        SmartDashboard.putBoolean("Intake SPINNING", this.getRollerState() != RollerState.DEFAULT);
    }

    @Override
    public void displayCommands() {
        SmartDashboard.putData("Intake IN", new SetIntakeState(IntakePosition.IN));
        SmartDashboard.putData("Intake OUT", new SetIntakeState(IntakePosition.OUT));
        SmartDashboard.putData("Intake Rollers IN", new SetRollersState(RollerState.IN));
        SmartDashboard.putData("Intake Rollers OUT", new SetRollersState(RollerState.OUT));
        SmartDashboard.putData("Intake Rollers NONE", new SetRollersState(RollerState.DEFAULT));
    }

    @Override
    public void initDefaultCommand() {
        // TODO Auto-generated method stub

    }
}