// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.Map;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/** Add your docs here. */
public class MotorTuner {
    /** Target PID parameters */
    TalonPIDConfig m_PIDConfig;

    /** Target motor to be configured */
    WPI_TalonFX m_motor;

    /** Network table entries for motor parameters */
    private NetworkTableEntry[] m_entries;

    /** Parameter listeners that receive updates from the dashboard */
    private ChangeDetector[] m_changeDetectors;

    private DoubleConsumer[] m_paramConsumers;

    /** Create an instance of the tuner for a given motor on the dashboard
     * @param title  Title assigned to the tuner
     * @param targetTab  Name of the tab where the tuner will be created
     */
    public MotorTuner(String title, ShuffleboardTab targetTab,
                      TalonPIDConfig targetConfig,
                      WPI_TalonFX targetMotor) {
        m_entries = new NetworkTableEntry[ParamID.numIDs.value];
        m_changeDetectors = new ChangeDetector[ParamID.numIDs.value];
        m_paramConsumers = new DoubleConsumer[ParamID.numIDs.value];
        m_PIDConfig = targetConfig;
        m_motor = targetMotor;

        // Initialize listeners
        m_paramConsumers[ParamID.id_kP.value] = 
            (d) -> { 
                m_PIDConfig.kP = d;
                m_motor.config_kP(m_PIDConfig.pidIndex, d, m_PIDConfig.timeoutMs);
            };
        m_paramConsumers[ParamID.id_kI.value] = 
            (d) -> {
                m_PIDConfig.kI = d;
                m_motor.config_kI(m_PIDConfig.pidIndex, d, m_PIDConfig.timeoutMs);
            };
        m_paramConsumers[ParamID.id_kD.value] =
            (d) -> {
                m_PIDConfig.kD = d;
                m_motor.config_kD(m_PIDConfig.pidIndex, d, m_PIDConfig.timeoutMs);
            };
        m_paramConsumers[ParamID.id_kF.value] =
            (d) -> {
                m_PIDConfig.kF = d;
                m_motor.config_kF(m_PIDConfig.pidIndex, d, m_PIDConfig.timeoutMs);
            };
        
        m_changeDetectors[ParamID.id_kP.value] = 
            new ChangeDetector(m_paramConsumers[ParamID.id_kP.value], targetConfig.kP);
        m_changeDetectors[ParamID.id_kI.value] = 
            new ChangeDetector(m_paramConsumers[ParamID.id_kI.value], targetConfig.kI);
        m_changeDetectors[ParamID.id_kD.value] = 
            new ChangeDetector(m_paramConsumers[ParamID.id_kD.value], targetConfig.kD);
        m_changeDetectors[ParamID.id_kF.value] = 
            new ChangeDetector(m_paramConsumers[ParamID.id_kF.value], targetConfig.kF);

        // Set up widgets and network table entries
        ShuffleboardLayout gainLayout = 
            targetTab.getLayout(title, BuiltInLayouts.kGrid)
            .withSize(1, ParamID.numIDs.value)
            .withProperties(
                Map.of("Label position", "TOP",
                       "Number of columns", 1,
                       "Number of rows", ParamID.numIDs.value));

        Map<String, Object> sliderProperties = 
            Map.of("min", 0.0, 
                   "max", 1.0,
                   "Block increment", 0.005);

        m_entries[ParamID.id_kP.value] = 
            gainLayout.add( "kP", targetConfig.kP)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(sliderProperties)
                .withPosition(0, ParamID.id_kP.value)
                .getEntry();

        m_entries[ParamID.id_kI.value] = 
            gainLayout.add( "kI", targetConfig.kI)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(sliderProperties)
                .withPosition(0, ParamID.id_kI.value)
                .getEntry();

        m_entries[ParamID.id_kD.value] = 
            gainLayout.add( "kD", targetConfig.kD)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(sliderProperties)
                .withPosition(0, ParamID.id_kD.value)
                .getEntry();

        m_entries[ParamID.id_kF.value] = 
            gainLayout.add( "kF", targetConfig.kF)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(sliderProperties)
                .withPosition(0, ParamID.id_kF.value)
                .getEntry();
    }

    /** Process changes in motor parameter values received from Shuffleboard */
    public boolean process() {
        boolean changed = false;
        for (int i = 0; i < m_changeDetectors.length; ++i) {
            changed |= m_changeDetectors[i].process(m_entries[i]);
        }
        return changed;
    }


    /** Indices of motor parameters */
    public enum ParamID {
        id_kP (0),
        id_kI (1),
        id_kD (2),
        id_kF (3),
        numIDs (4);

        public final int value;
        private ParamID(int val) {this.value = val;}
    }

    /** Inner class used to check for a change in the value of a network event
     *  table entry for a motor parameter.
    */
    private class ChangeDetector {
        private DoubleConsumer m_listener;
        private double m_value;

        /** Creates an instance of the object and sets the listener to be
         *  notified when the object's value changes.
         * 
         * @param listener   Object to be notified when the parameter value changes
         * @param initialValue Initial value for the parameter
         */
        public ChangeDetector(DoubleConsumer listener, double initialValue) {
            m_listener = listener;
            m_value = initialValue;
        }

        /** Process the current value of a given NetworkTableEntry and return true
         *  if it changed
         */
        public boolean process(NetworkTableEntry entry) {
            boolean changed = false;
            double val = entry.getDouble(m_value);
            if (val != m_value) {
                m_listener.accept(val);
                m_value = val;
                changed = true;
            }
            return changed;
        }
    }
}
