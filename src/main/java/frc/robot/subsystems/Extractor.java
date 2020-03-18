package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team3316.kit.DBugSubsystem;
import com.team3316.kit.motors.DBugTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.extractor.SetExtractorState;
import frc.robot.utils.Utils;

/**
 * Extractor
 */
public class Extractor extends DBugSubsystem {
    private DBugTalon _roller;
    private ExtractorState _currentState = ExtractorState.NONE;

    public Extractor() {
        this._roller = (DBugTalon) Utils.getBean("extractor.roller");

        this._roller.setInverted(Constants.Extractor.RollerVictor.sensorPhase);
        this._roller.setSensorPhase(Constants.Extractor.RollerVictor.inverted);

        this._roller.setDistancePerRevolution(Constants.Extractor.RollerVictor.dpr, Constants.Sensors.Encoders.Quad.upr);

        // this._roller.setupPIDF(Constants.Extractor.RollerVictor.PIDF.kP,
        //         Constants.Extractor.RollerVictor.PIDF.kI,
        //         Constants.Extractor.RollerVictor.PIDF.kD,
        //         Constants.Extractor.RollerVictor.PIDF.kF);

        // this._roller.configAllowableClosedloopError(0, Constants.Extractor.RollerVictor.tolerance,
        //     0);

        this._roller.configVoltageCompSaturation(Constants.Extractor.RollerVictor.voltageCompensation);
        this._roller.enableVoltageCompensation(true);
    }

    public enum ExtractorState {
        IN(Constants.Extractor.RollerState.IN.percentage),
        OUT(Constants.Extractor.RollerState.OUT.percentage),
        NONE(Constants.Extractor.RollerState.NONE.percentage);

        private double _percentage;
        
        private ExtractorState(double percentage) {
            this._percentage = percentage;
        }

        public double getPercentage() {
            return this._percentage;
        }

    }

    /**
     *
     * @param state - <code>ExtractorState</code> to set to
     */
    public void setExtractorState(ExtractorState state) {
        this._currentState = state;
        this._roller.set(ControlMode.PercentOutput, state.getPercentage());
    }

    /**
     *
     * @return current <code>ExtractorState</code>
     */
    public ExtractorState getExtractorState() {
        return this._currentState;
    }

    public double getCurrent() {
        return this._roller.getStatorCurrent();
    }

    @Override
    public void initDefaultCommand() {
        // Nothing
    }

    @Override
    public void displayTestData() {
        SmartDashboard.putBoolean("Extractor ROTATING", this.getExtractorState() != ExtractorState.NONE);
        SmartDashboard.putNumber("Extractor SPEED", this._roller.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Extractor Current", this._roller.getOutputCurrent());
    }

    @Override
    public void displayMatchData() {
        // Nothing
    }

    @Override
    public void displayCommands() {
        SmartDashboard.putData("Extractor IN", new SetExtractorState(ExtractorState.IN));
        SmartDashboard.putData("Extractor OUT", new SetExtractorState(ExtractorState.OUT));
        SmartDashboard.putData("Extractor NONE", new SetExtractorState(ExtractorState.NONE));
    }
}