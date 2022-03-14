// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.fireBallSubcommands;


/** This class implements detection of a pulse as a series of transitions in a boolean
 * input:
 * 
 *  .   . ------------   .    .  TRUE
 *        ^          |
 *        |          |
 *        |          |
 *  ______|          v__________ FALSE
 * 
 *  FALSE --> TRUE --> FALSE
 */
public class PulseDetector {

    private enum State {
        DetectRisingEdge,
        DetectFallingEdge
    }

    private boolean m_lastInput;
    private State m_state;

    ///////////////////////////////////////////////////////////////////////////
    /** Creates an instance of the object and sets it up to detect a pulse
     */
    PulseDetector() { 
        reset();
    }

    ///////////////////////////////////////////////////////////////////////////
    /** Process the present input value
     * 
     * @param input  The present input value
     * @return true if a pulse has been detected; else false
     * @remarks
     *   Once a complete pulse is detected, the detector is automatically reset
     *   to detect a subsequent pulse
     */
    public boolean process(boolean input) {
        boolean pulseDetected = false;

        switch (m_state) {
            case DetectRisingEdge: {
                if (!m_lastInput && input) {
                    m_state = State.DetectFallingEdge;
                }
                break;
            }

            case DetectFallingEdge: {
                if (m_lastInput && !input) {
                    pulseDetected = true;
                    m_state = State.DetectRisingEdge;
                }
                break;
            }
        }

        m_lastInput = input;
        return pulseDetected;
    }

    ///////////////////////////////////////////////////////////////////////////
    /** Reset the detector and make it ready to detect another pulse
    */
    public void reset() {
        m_lastInput = false;
        m_state = State.DetectRisingEdge;
    }
}