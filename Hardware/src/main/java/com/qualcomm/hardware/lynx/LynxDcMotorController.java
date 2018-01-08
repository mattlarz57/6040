/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.hardware.lynx;

import android.content.Context;

import com.qualcomm.hardware.R;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelEnableResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorChannelModeResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorConstantPowerResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorEncoderPositionResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorPIDControlLoopCoefficientsResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetPositionResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetResponse;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelEnableCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorChannelModeCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDControlLoopCoefficientsCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetPositionCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorTargetVelocityCommand;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.ExpansionHubMotorControllerParamsState;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * LynxDcMotorController implements motor controller semantics on Lynx module
 */
@SuppressWarnings("WeakerAccess")
public class LynxDcMotorController extends LynxController implements DcMotorController, DcMotorControllerEx
    {
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public static final int apiMotorFirst = LynxConstants.INITIAL_MOTOR_PORT;
    public static final int apiMotorLast = apiMotorFirst + LynxConstants.NUMBER_OF_MOTORS -1;
    public static final double apiPowerFirst = -1.0;
    public static final double apiPowerLast  = 1.0;

    public static final String TAG = "LynxMotor";
    @Override protected String getTag() { return TAG; }

    protected static boolean DEBUG = false;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected class MotorProperties
        {
        // We have caches of values that we *could* read from the controller, and need to
        // do so if the cache is invalid
        LastKnown<Double>                       lastKnownPower              = new LastKnown<Double>();
        LastKnown<Integer>                      lastKnownTargetPosition     = new LastKnown<Integer>();
        LastKnown<DcMotor.RunMode>              lastKnownMode               = new LastKnown<DcMotor.RunMode>();
        LastKnown<DcMotor.ZeroPowerBehavior>    lastKnownZeroPowerBehavior  = new LastKnown<DcMotor.ZeroPowerBehavior>();
        LastKnown<Boolean>                      lastKnownEnable             = new LastKnown<Boolean>();

        // The remainder of the data is authoritative, here
        MotorConfigurationType                  motorType = MotorConfigurationType.getUnspecifiedMotorType();
        Map<DcMotor.RunMode, ExpansionHubMotorControllerParamsState> desiredPIDParams = new ConcurrentHashMap<DcMotor.RunMode, ExpansionHubMotorControllerParamsState>();
        }

    // this is indexed from zero, not 1 as it is in the legacy and modern motor controllers
    protected final MotorProperties[] motors = new MotorProperties[LynxConstants.NUMBER_OF_MOTORS];

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public LynxDcMotorController(final Context context, final LynxModule module)
            throws RobotCoreException, InterruptedException
        {
        super(context, module);
        for (int motor = 0; motor < motors.length; motor++)
            {
            motors[motor] = new MotorProperties();
            }
        this.finishConstruction();
        }

    @Override public void initializeHardware() throws RobotCoreException, InterruptedException
        {
        floatHardware();
        runWithoutEncoders();
        //
        forgetLastKnown();
        //
        for (int motor = 0; motor <= apiMotorLast-apiMotorFirst; motor++)
            {
            updateMotorParams(motor);
            }
        reportPIDControlLoopCoefficients();
        }

    //----------------------------------------------------------------------------------------------
    // Arming and disarming
    //----------------------------------------------------------------------------------------------

    @Override protected void doHook()
        {
        forgetLastKnown();
        }

    @Override protected void doUnhook()
        {
        forgetLastKnown();
        }

    @Override public void forgetLastKnown()
        {
        for (MotorProperties motor : this.motors)
            {
            motor.lastKnownMode.invalidate();
            motor.lastKnownPower.invalidate();
            motor.lastKnownTargetPosition.invalidate();
            motor.lastKnownZeroPowerBehavior.invalidate();
            motor.lastKnownEnable.invalidate();
            }
        }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice interface
    //----------------------------------------------------------------------------------------------

    @Override
    public String getDeviceName()
        {
        return this.context.getString(R.string.lynxDcMotorControllerDisplayName);
        }

    //----------------------------------------------------------------------------------------------
    // DcMotorControllerEx interface
    //----------------------------------------------------------------------------------------------

    @Override public synchronized void setMotorEnable(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        internalSetMotorEnable(motor, true);
        }

    @Override public synchronized void setMotorDisable(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        internalSetMotorEnable(motor, false);
        }

    void internalSetMotorEnable(int motorZ, boolean enable)
        {
        if (motors[motorZ].lastKnownEnable.updateValue(enable))
            {
            LynxCommand command = new LynxSetMotorChannelEnableCommand(this.getModule(), motorZ, enable);
            try {
                if (DEBUG) RobotLog.vv(TAG,"setMotorEnable mod=%d motor=%d enable=%s", getModuleAddress(), motorZ, ((Boolean) enable).toString());
                command.send();
                }
            catch (InterruptedException|RuntimeException|LynxNackException e)
                { 
                handleException(e); 
                }
            }
        }

    @Override public synchronized boolean isMotorEnabled(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;

        Boolean result = motors[motor].lastKnownEnable.getValue();
        if (result != null)
            {
            return result;
            }

        LynxGetMotorChannelEnableCommand command = new LynxGetMotorChannelEnableCommand(this.getModule(), motor);
        try {
            LynxGetMotorChannelEnableResponse response = command.sendReceive();
            result = response.isEnabled();
            motors[motor].lastKnownEnable.setValue(result);
            return result;
            }
        catch (InterruptedException|RuntimeException|LynxNackException e) 
            { 
            handleException(e); 
            }
        return LynxUsbUtil.makePlaceholderValue(true);
        }

    //----------------------------------------------------------------------------------------------
    // DcMotorController interface
    //----------------------------------------------------------------------------------------------

    @Override public synchronized MotorConfigurationType getMotorType(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        return motors[motor].motorType;
        }

    @Override public synchronized void setMotorType(int motor, MotorConfigurationType motorType)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        motors[motor].motorType = motorType;

        // Remember parameterization, overriding any user-specified values.
        if (motorType.hasExpansionHubVelocityParams())
            {
            rememberPIDParams(motor, motorType.getHubVelocityParams());
            }
        if (motorType.hasExpansionHubPositionParams())
            {
            rememberPIDParams(motor, motorType.getHubPositionParams());
            }
        updateMotorParams(motor);
        }

    protected void rememberPIDParams(int motorZ, ExpansionHubMotorControllerParamsState params)
        {
        motors[motorZ].desiredPIDParams.put(params.mode, params);
        }

    protected void updateMotorParams(int motorZ)
        {
        for (ExpansionHubMotorControllerParamsState params : motors[motorZ].desiredPIDParams.values())
            {
            if (!params.isDefault())
                {
                internalSetPIDCoefficients(motorZ, params.mode, params.p, params.i, params.d);
                }
            }
        }

    protected int getDefaultMaxMotorSpeed(int motorZ)
        {
        return motors[motorZ].motorType.getAchieveableMaxTicksPerSecondRounded();
        }

    @Override public synchronized void setMotorMode(int motor, DcMotor.RunMode mode)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        // We don't updateValue() the mode until it's actually changed so that reading modes
        // and power levels will be interpreted correctly.
        if (!motors[motor].lastKnownMode.isValue(mode))
            {
            // Get the current power so we can preserve across the change.
            Double prevPower = motors[motor].lastKnownPower.getNonTimedValue();
            if (prevPower == null)
                {
                prevPower = internalGetMotorPower(motor);
                }

            LynxCommand command;
            DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
            if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER)
                {
                // Stop the motor, but not in such a way that we disrupt the last known
                // power, since we need to restore same when we come out of this mode.
                internalSetMotorPower(motor, 0);
                command = new LynxResetMotorEncoderCommand(this.getModule(), motor);
                }
            else
                {
                zeroPowerBehavior = internalGetZeroPowerBehavior(motor);
                command = new LynxSetMotorChannelModeCommand(this.getModule(), motor, mode, zeroPowerBehavior);
                }
            try {
                if (DEBUG) RobotLog.vv(TAG, "setMotorChannelMode: mod=%d motor=%d mode=%s power=%f zero=%s",
                                       getModuleAddress(), motor, mode.toString(), prevPower, zeroPowerBehavior.toString());
                command.send();

                // Ok, remember that mode. Note we need to set it before we call internalSetMotorPower()
                motors[motor].lastKnownMode.setValue(mode);

                // re-issue current motor power to ensure it's correct for this mode
                internalSetMotorPower(motor, prevPower, true);
                }
            catch (InterruptedException|RuntimeException|LynxNackException e) 
                { 
                handleException(e); 
                }
            }
        }

    @Override public synchronized DcMotor.RunMode getMotorMode(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        return internalGetPublicMotorMode(motor);
        }

    /** @see #internalGetMotorChannelMode(int) */
    protected DcMotor.RunMode internalGetPublicMotorMode(int motorZ)
        {
        // Do we have a cached answer?
        DcMotor.RunMode result = motors[motorZ].lastKnownMode.getValue();
        if (result != null)
            {
            return result;
            }

        // STOP_AND_RESET_ENCODER doesn't expire
        if (motors[motorZ].lastKnownMode.getNonTimedValue()== DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            {
            return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            }

        // We don't actually know, or the value isn't fresh: ask the hardware
        LynxGetMotorChannelModeCommand command = new LynxGetMotorChannelModeCommand(this.getModule(), motorZ);
        try {
            LynxGetMotorChannelModeResponse response = command.sendReceive();
            result = response.getMode();
            motors[motorZ].lastKnownMode.setValue(result);
            return result;
            }
        catch (InterruptedException|RuntimeException|LynxNackException e) 
            { 
            handleException(e); 
            }
        return LynxUsbUtil.makePlaceholderValue(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    /**
     * like {@link #internalGetPublicMotorMode(int)}, but ALWAYS returns a value that can be set
     * with {@link LynxSetMotorChannelModeCommand}. We also don't here update lastKnownMode.
     * @see #internalGetPublicMotorMode(int)
     */
    protected DcMotor.RunMode internalGetMotorChannelMode(int motorZ)
        {
        DcMotor.RunMode result = motors[motorZ].lastKnownMode.getValue();
        if (result != null && result != DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            {
            return result;
            }

        LynxGetMotorChannelModeCommand command = new LynxGetMotorChannelModeCommand(this.getModule(), motorZ);
        try {
            LynxGetMotorChannelModeResponse response = command.sendReceive();
            result = response.getMode();
            return result;
            }
        catch (InterruptedException|RuntimeException|LynxNackException e) 
            { 
            handleException(e); 
            }
        return LynxUsbUtil.makePlaceholderValue(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    @Override public synchronized void setMotorPower(int motor, double apiMotorPower)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        internalSetMotorPower(motor, apiMotorPower);
        }

    @Override public synchronized double getMotorPower(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        return internalGetMotorPower(motor);
        }

    DcMotor.ZeroPowerBehavior internalGetZeroPowerBehavior(int motorZ)
        {
        // Do we have a cached answer?
        DcMotor.ZeroPowerBehavior result = motors[motorZ].lastKnownZeroPowerBehavior.getValue();
        if (result != null)
            {
            return result;
            }

        // No, actually talk to the hardware
        LynxGetMotorChannelModeCommand command = new LynxGetMotorChannelModeCommand(this.getModule(), motorZ);
        try {
            LynxGetMotorChannelModeResponse response = command.sendReceive();
            result = response.getZeroPowerBehavior();
            motors[motorZ].lastKnownZeroPowerBehavior.setValue(result);
            return result;
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    void internalSetZeroPowerBehavior(int motorZ, DcMotor.ZeroPowerBehavior behavior)
        {
        if (motors[motorZ].lastKnownZeroPowerBehavior.updateValue(behavior))
            {
            DcMotor.RunMode runMode = internalGetMotorChannelMode(motorZ);
            LynxCommand command = new LynxSetMotorChannelModeCommand(this.getModule(), motorZ, runMode, behavior);
            try {
                if (DEBUG) RobotLog.vv(TAG,"setZeroBehavior mod=%d motor=%d zero=%s",
                                       getModuleAddress(), motorZ, behavior.toString());
                command.send();
                }
            catch (InterruptedException|RuntimeException|LynxNackException e)
                {
                handleException(e);
                }
            }
        }

    void internalSetMotorPower(int motorZ, double apiPower)
        {
        internalSetMotorPower(motorZ, apiPower, false);
        }

    void internalSetMotorPower(int motorZ, double apiPower, boolean forceUpdate)
        {
        double power = Range.clip(apiPower, apiPowerFirst, apiPowerLast);
        int iPower = 0;
        if (motors[motorZ].lastKnownPower.updateValue(power) || forceUpdate)
            {
            DcMotor.RunMode mode = internalGetPublicMotorMode(motorZ);
            LynxCommand command = null;
            switch (mode)
                {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER:
                    {
                    // Scale 'power' to configured maximum motor speed. This is mostly for legacy
                    // compatibility, as setMotorVelocity exposes this more directly.
                    power = Math.signum(power) * Range.scale(Math.abs(power), 0, apiPowerLast, 0, getDefaultMaxMotorSpeed(motorZ));
                    iPower = (int)power;
                    command = new LynxSetMotorTargetVelocityCommand(this.getModule(), motorZ, iPower);
                    break;
                    }
                case RUN_WITHOUT_ENCODER:
                    {
                    power = Range.scale(power, apiPowerFirst, apiPowerLast, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast);
                    iPower = (int)power;
                    command = new LynxSetMotorConstantPowerCommand(this.getModule(), motorZ, iPower);
                    break;
                    }
                case STOP_AND_RESET_ENCODER:
                    {
                    // Setting motor power in this mode doesn't do anything
                    command = null;
                    break;
                    }
                }
            try {
                if (command != null)
                    {
                    if (DEBUG) RobotLog.vv(TAG, "setMotorPower: mod=%d motor=%d iPower=%d", getModuleAddress(), motorZ, iPower);
                    command.send();
                    internalSetMotorEnable(motorZ, true);
                    }
                }
            catch (InterruptedException|RuntimeException|LynxNackException e)
                {
                handleException(e);
                }
            }
        }

    double internalGetMotorPower(int motorZ)
        {
        // Do we have a cached answer?
        Double result = motors[motorZ].lastKnownPower.getValue();
        if (result != null)
            {
            if (DEBUG) RobotLog.vv(TAG, "getMotorPower(cached): mod=%d motor=%d power=%f", getModuleAddress(), motorZ, result);
            return result;
            }

        // No, actually talk to the hardware
        DcMotor.RunMode mode = internalGetPublicMotorMode(motorZ);
        try {
            switch (mode)
                {
                case RUN_TO_POSITION:
                case RUN_USING_ENCODER:
                    {
                    LynxGetMotorTargetVelocityCommand command = new LynxGetMotorTargetVelocityCommand(this.getModule(), motorZ);
                    LynxGetMotorTargetVelocityResponse response = command.sendReceive();

                    // Scale relative to the *current* max speed
                    int iVelocity = response.getVelocity();
                    result = Math.signum(iVelocity) * Range.scale(Math.abs(iVelocity), 0, getDefaultMaxMotorSpeed(motorZ), 0, apiPowerLast);
                    if (DEBUG) RobotLog.vv(TAG, "getMotorPower: mod=%d motor=%d velocity=%d power=%f", getModuleAddress(), motorZ, iVelocity, result);
                    break;
                    }
                case RUN_WITHOUT_ENCODER:
                default:
                    {
                    LynxGetMotorConstantPowerCommand command = new LynxGetMotorConstantPowerCommand(this.getModule(), motorZ);
                    LynxGetMotorConstantPowerResponse response = command.sendReceive();
                    int iPower = response.getPower();
                    result = Range.scale(iPower, LynxSetMotorConstantPowerCommand.apiPowerFirst, LynxSetMotorConstantPowerCommand.apiPowerLast, apiPowerFirst, apiPowerLast);
                    if (DEBUG) RobotLog.vv(TAG, "getMotorPower: mod=%d motor=%d iPower=%d power=%f", getModuleAddress(), motorZ, iPower, result);
                    }
                }
            result = Range.clip(result, apiPowerFirst, apiPowerLast);    // paranoia
            motors[motorZ].lastKnownPower.setValue(result);
            // if (DEBUG) RobotLog.vv(TAG, "getMotorPower: mod=%d motor=%d power=%f", getModuleAddress(), motorZ, result);
            return result;
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(0);
        }

    @Override public synchronized boolean isBusy(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        LynxIsMotorAtTargetCommand command = new LynxIsMotorAtTargetCommand(this.getModule(), motor);
        try {
            LynxIsMotorAtTargetResponse response = command.sendReceive();
            return !response.isAtTarget();  // isBusy is true when motor is not at target position.
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(false); // lacking data better to unstick otherwise-infinite typical wait loops by saying *not* busy
        }

    @Override public synchronized void setMotorZeroPowerBehavior(int motor, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        if (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.UNKNOWN) throw new IllegalArgumentException("zeroPowerBehavior may not be UNKNOWN");

        internalSetZeroPowerBehavior(motor, zeroPowerBehavior);
        }

    @Override public synchronized DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        return internalGetZeroPowerBehavior(motor);
        }

    protected synchronized void setMotorPowerFloat(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        internalSetZeroPowerBehavior(motor, DcMotor.ZeroPowerBehavior.FLOAT);
        internalSetMotorPower(motor, 0);
        }

    @Override public synchronized boolean getMotorPowerFloat(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        return internalGetZeroPowerBehavior(motor) == DcMotor.ZeroPowerBehavior.FLOAT
                && internalGetMotorPower(motor) == 0;
        }

    @Override public synchronized void setMotorTargetPosition(int motor, int position)
        {
        setMotorTargetPosition(motor, position, LynxConstants.DEFAULT_TARGET_POSITION_TOLERANCE);
        }

    @Override public synchronized void setMotorTargetPosition(int motor, int position, int tolerance)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        LynxSetMotorTargetPositionCommand command = new LynxSetMotorTargetPositionCommand(this.getModule(), motor, position, tolerance);
        try {
            command.send();
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        }

    @Override public synchronized int getMotorTargetPosition(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        LynxGetMotorTargetPositionCommand command = new LynxGetMotorTargetPositionCommand(this.getModule(), motor);
        try {
            LynxGetMotorTargetPositionResponse response = command.sendReceive();
            return response.getTarget();
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(0);
        }

    @Override public synchronized int getMotorCurrentPosition(int motor)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        LynxGetMotorEncoderPositionCommand command = new LynxGetMotorEncoderPositionCommand(this.getModule(), motor);
        try {
            LynxGetMotorEncoderPositionResponse response = command.sendReceive();
            return response.getPosition();
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(0);
        }

    @Override public synchronized void setMotorVelocity(int motor, double angularRate, AngleUnit unit)
        {
        // If we're setting a target velocity, then we have to be in velocity mode, so put us there
        // if we aren't already (the alternative would have been to throw an error, which seems pointless).
        setMotorMode(motor, DcMotor.RunMode.RUN_USING_ENCODER);

        this.validateMotor(motor); motor -= apiMotorFirst;

        double degreesPerSecond     = UnnormalizedAngleUnit.DEGREES.fromUnit(unit.getUnnormalized(), angularRate);
        double revolutionsPerSecond = degreesPerSecond / 360.0;
        double ticksPerSecond       = motors[motor].motorType.getTicksPerRev() * revolutionsPerSecond;

        int iTicksPerSecond = Range.clip((int)Math.round(ticksPerSecond),
                LynxSetMotorTargetVelocityCommand.apiVelocityFirst,
                LynxSetMotorTargetVelocityCommand.apiVelocityLast);

        try {
            LynxCommand command = new LynxSetMotorTargetVelocityCommand(this.getModule(), motor, iTicksPerSecond);
            if (DEBUG) RobotLog.vv(TAG, "setMotorVelocity: mod=%d motor=%d iPower=%d", getModuleAddress(), motor, iTicksPerSecond);
            command.send();
            internalSetMotorEnable(motor, true);
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        }

    @Override public synchronized double getMotorVelocity(int motor, AngleUnit unit)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        int ticksPerSecond = internalGetMotorTicksPerSecond(motor);
        double revsPerSecond = ticksPerSecond / motors[motor].motorType.getTicksPerRev();
        return unit.getUnnormalized().fromDegrees(revsPerSecond * 360.0);
        }

    int internalGetMotorTicksPerSecond(int motorZ)
        {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(this.getModule());
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            return response.getVelocity(motorZ);      // in encoder counts per second
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(0);
        }

    @Override public synchronized void setPIDCoefficients(int motor, DcMotor.RunMode mode, PIDCoefficients pidCoefficients)
        {
        this.validatePIDMode(motor, mode);
        this.validateMotor(motor); motor -= apiMotorFirst;

        // Allow old names for modes, but only on incoming APIs
        mode = mode.migrate();

        // Remember that we've overridden the values specified by the motor type so that we don't
        // mistakenly undo this effect in updateMotorParams()
        rememberPIDParams(motor, new ExpansionHubMotorControllerParamsState(mode, pidCoefficients.p, pidCoefficients.i, pidCoefficients.d));

        // Actually change the values
        internalSetPIDCoefficients(motor, mode, pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        }

    protected void internalSetPIDCoefficients(int motorZ, DcMotor.RunMode mode, double pp, double ii, double dd)
        {
        int p = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(pp);
        int i = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(ii);
        int d = LynxSetMotorPIDControlLoopCoefficientsCommand.internalCoefficientFromExternal(dd);
        LynxSetMotorPIDControlLoopCoefficientsCommand command = new LynxSetMotorPIDControlLoopCoefficientsCommand(this.getModule(), motorZ, mode, p, i, d);
        try {
            command.send();
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        }

    @Override public synchronized PIDCoefficients getPIDCoefficients(int motor, DcMotor.RunMode mode)
        {
        this.validateMotor(motor); motor -= apiMotorFirst;
        LynxGetMotorPIDControlLoopCoefficientsCommand command = new LynxGetMotorPIDControlLoopCoefficientsCommand(this.getModule(), motor, mode);
        try {
            LynxGetMotorPIDControlLoopCoefficientsResponse response = command.sendReceive();
            return new PIDCoefficients(
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getP()),
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getI()),
                    LynxSetMotorPIDControlLoopCoefficientsCommand.externalCoefficientFromInternal(response.getD())
                );
            }
        catch (InterruptedException|RuntimeException|LynxNackException e)
            {
            handleException(e);
            }
        return LynxUsbUtil.makePlaceholderValue(new PIDCoefficients());
        }

    //------------------------------------------------------------------------------------------------
    // Utility
    //------------------------------------------------------------------------------------------------

    @Override public void floatHardware()
        {
        for (int motor = apiMotorFirst; motor <= apiMotorLast; motor++)
            {
            setMotorPowerFloat(motor);
            }
        }

    private void runWithoutEncoders()
        {
        for (int motor = apiMotorFirst; motor <= apiMotorLast; motor++)
            {
            setMotorMode(motor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

    private void validateMotor(int motor)
        {
        if (motor < apiMotorFirst || motor > apiMotorLast)
            {
            throw new IllegalArgumentException(String.format("motor %d is invalid; valid motors are %d..%d", motor, apiMotorFirst, apiMotorLast));
            }
        }

    private void validatePIDMode(int motor, DcMotor.RunMode runMode)
        {
        if (!runMode.isPIDMode())
            {
            throw new IllegalArgumentException(String.format("motor %d: mode %s is invalid as PID Mode", motor, runMode));
            }
        }

    private void reportPIDControlLoopCoefficients() throws RobotCoreException, InterruptedException
        {
        reportPIDControlLoopCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        reportPIDControlLoopCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    private void reportPIDControlLoopCoefficients(DcMotor.RunMode mode) throws RobotCoreException, InterruptedException
        {
        PIDCoefficients coeffs;

        for (int motor = apiMotorFirst; motor <= apiMotorLast; motor++)
            {
            coeffs = getPIDCoefficients(motor,mode);
            if (DEBUG) RobotLog.vv(TAG,"mod=%d motor=%d mode=%s p=%f i=%f d=%f",
                                   getModuleAddress(), motor, mode.toString(), coeffs.p, coeffs.i, coeffs.d);
            }
        }

    protected int getModuleAddress()
        {
        return this.getModule().getModuleAddress();
        }
    }
