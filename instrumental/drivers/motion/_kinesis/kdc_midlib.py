# -*- coding: utf-8 -*-
# Copyright 2017-2018 Nate Bogdanowicz

from nicelib import load_lib, NiceLib, Sig, NiceObject, RetHandler, ret_return, ret_ignore
# from nicelib import NiceLib, Sig, NiceObject, RetHandler, ret_return, ret_ignore
from .common_midlib import ret_errcheck, ret_success, all_sigs
from ....errors import Error

class NiceKDC(NiceLib):
    """Mid-level wrapper for Thorlabs.MotionControl.KCube.DCServo.dll"""
    _info_ = load_lib('kdc_', __package__, builder='._build_kinesis',
                      kwargs={'shortname': 'kdc',
                              'sublib': 'Thorlabs.MotionControl.KCube.DCServo'})
    _prefix_ = 'TLI_'
    _ret_ = ret_errcheck

    # BuildDeviceList = Sig()
    # GetDeviceListSize = Sig(ret=ret_return)
    # GetDeviceListExt = Sig('buf', 'len')
    # GetDeviceListByTypeExt = Sig('buf', 'len', 'in')
    # GetDeviceListByTypesExt = Sig('buf', 'len', 'in', 'in')
    # GetDeviceInfo = Sig('in', 'out', ret=ret_success)
    BuildDeviceList = Sig()
    GetDeviceListSize = Sig(ret=ret_return)
    GetDeviceInfo = Sig('in', 'out', ret=ret_success)
    GetDeviceList = Sig('out')
    GetDeviceListByType = Sig('out', 'in')
    GetDeviceListByTypes = Sig('out', 'in', 'in')
    GetDeviceListExt = Sig('buf', 'len')
    GetDeviceListByTypeExt = Sig('buf', 'len', 'in')
    GetDeviceListByTypesExt = Sig('buf', 'len', 'in', 'in')
    InitializeSimulations = Sig(ret=ret_ignore)
    UninitializeSimulations = Sig(ret=ret_ignore)
    # GetDeviceList = Sig('out')
    # GetDeviceListByType = Sig('out', 'in', dict(first_arg=False))
    # GetDeviceListByTypes = Sig('out', 'in', 'in', dict(first_arg=False))
    class Device(NiceObject):
        _prefix_ = 'CC_'
        _sigs_ = {
            'Open' : Sig('in'),
            'Close' : Sig('in', ret=ret_return),
            'Identify' : Sig('in', ret=ret_return),
            'GetLEDswitches' : Sig('in', ret=ret_return),
            'SetLEDswitches' : Sig('in', 'in'),
            'GetHardwareInfo' : Sig('in', 'buf', 'len', 'out', 'out', 'buf', 'len', 'out', 'out', 'out'),
            'GetHardwareInfoBlock' : Sig('in', 'out'),
            'GetHubBay' : Sig('in', ret=ret_return),
            'GetSoftwareVersion' : Sig('in', ret=ret_return),
            'LoadSettings' : Sig('in', ret=ret_success),
            'PersistSettings' : Sig('in', ret=ret_success),
            'DisableChannel' : Sig('in'),
            'EnableChannel' : Sig('in'),
            'GetNumberPositions' : Sig('in', ret=ret_return),
            'CanHome' : Sig('in', ret=ret_success),
            'NeedsHoming' : Sig('in', ret=ret_return),
            'Home' : Sig('in'),
            'MoveToPosition' : Sig('in', 'in'),
            'GetPosition' : Sig('in', ret=ret_return),
            'GetHomingVelocity' : Sig('in', ret=ret_return),
            'SetHomingVelocity' : Sig('in', 'in'),
            'MoveRelative' : Sig('in', 'in'),
            'GetJogMode' : Sig('in', 'out', 'out'),
            'SetJogMode' : Sig('in', 'in', 'in'),
            'SetJogStepSize' : Sig('in', 'in'),
            'GetJogStepSize' : Sig('in', ret=ret_return),
            'GetJogVelParams' : Sig('in', 'out', 'out'),
            'SetJogVelParams' : Sig('in', 'in', 'in'),
            'MoveJog' : Sig('in', 'in'),
            'SetVelParams' : Sig('in', 'in', 'in'),
            'GetVelParams' : Sig('in', 'out', 'out'),
            'MoveAtVelocity' : Sig('in', 'in'),
            'SetDirection' : Sig('in', 'in', ret=ret_return),
            'StopImmediate' : Sig('in'),
            'StopProfiled' : Sig('in'),
            'GetBacklash' : Sig('in', ret=ret_return),
            'SetBacklash' : Sig('in', 'in'),
            'GetPositionCounter' : Sig('in', ret=ret_return),
            'SetPositionCounter' : Sig('in', 'in'),
            'GetEncoderCounter' : Sig('in', ret=ret_return),
            'SetEncoderCounter' : Sig('in', 'in'),
            'GetLimitSwitchParams' : Sig('in', 'out', 'out', 'out', 'out', 'out'),
            'SetLimitSwitchParams' : Sig('in', 'in', 'in', 'in', 'in', 'in'),
            'GetSoftLimitMode' : Sig('in', ret=ret_return),
            'SetLimitsSoftwareApproachPolicy' : Sig('in', 'in', ret=ret_return),
            'GetButtonParams' : Sig('in', 'out', 'out', 'out', 'out'),
            'SetButtonParams' : Sig('in', 'in', 'in', 'in'),
            'SetPotentiometerParams' : Sig('in', 'in', 'in', 'in'),
            'GetPotentiometerParams' : Sig('in', 'in', 'out', 'out'),
            'GetVelParamsBlock' : Sig('in', 'out'),
            'SetVelParamsBlock' : Sig('in', 'in'),
            'SetMoveAbsolutePosition' : Sig('in', 'in'),
            'GetMoveAbsolutePosition' : Sig('in', ret=ret_return),
            'MoveAbsolute' : Sig('in'),
            'SetMoveRelativeDistance' : Sig('in', 'in'),
            'GetMoveRelativeDistance' : Sig('in', ret=ret_return),
            'MoveRelativeDistance' : Sig('in'),
            'GetHomingParamsBlock' : Sig('in', 'out'),
            'SetHomingParamsBlock' : Sig('in', 'in'),
            'GetJogParamsBlock' : Sig('in', 'out'),
            'SetJogParamsBlock' : Sig('in', 'in'),
            'GetButtonParamsBlock' : Sig('in', 'out'),
            'SetButtonParamsBlock' : Sig('in', 'in'),
            'GetPotentiometerParamsBlock' : Sig('in', 'out'),
            'SetPotentiometerParamsBlock' : Sig('in', 'in'),
            'GetLimitSwitchParamsBlock' : Sig('in', 'out'),
            'SetLimitSwitchParamsBlock' : Sig('in', 'in'),
            'GetDCPIDParams' : Sig('in', 'out'),
            'SetDCPIDParams' : Sig('in', 'in'),
            'SuspendMoveMessages' : Sig('in'),
            'ResumeMoveMessages' : Sig('in'),
            'RequestPosition' : Sig('in'),
            'RequestStatusBits' : Sig('in', ret=ret_return),
            'GetStatusBits' : Sig('in', ret=ret_return),
            'StartPolling' : Sig('in', 'in', ret=ret_success),
            'PollingDuration' : Sig('in', ret=ret_return),
            'StopPolling' : Sig('in', ret=ret_return),
            'RequestSettings' : Sig('in'),
            'GetStageAxisMinPos' : Sig('in', ret=ret_return),
            'GetStageAxisMaxPos' : Sig('in', ret=ret_return),
            'SetStageAxisLimits' : Sig('in', 'in', 'in'),
            'SetMotorTravelMode' : Sig('in', 'in'),
            'GetMotorTravelMode' : Sig('in', ret=ret_return),
            'SetMotorParams' : Sig('in', 'in', 'in', 'in'),
            'GetMotorParams' : Sig('in', 'out', 'out', 'out'),
            'SetMotorParamsExt' : Sig('in', 'in', 'in', 'in'),
            'GetMotorParamsExt' : Sig('in', 'out', 'out', 'out'),
            'ClearMessageQueue' : Sig('in', ret=ret_return),
            'RegisterMessageCallback' : Sig('in', 'in', ret=ret_return),
            'MessageQueueSize' : Sig('in', ret=ret_return),
            'GetNextMessage' : Sig('in', 'out', 'out', 'out', ret=ret_success),
            # 'WaitForMessage' : Sig('in', 'in', 'in', 'in', ret=ret_success),
            'WaitForMessage': Sig('in', 'out', 'out', 'out', ret=ret_success),
        }


# @RetHandler(num_retvals=0)
# def ret_success(success):
#     if not success:
#         raise KDC101Error('The function did not execute successfully')
#
#
# @RetHandler(num_retvals=0)
# def ret_errcheck(retval):
#     if not (retval == 0 or retval is None):
#         raise KDC101Error(error_dict[retval])
#
# class KDC101Error(Error):
#     pass


# error_dict = {
#     0: 'OK - Success  ',
#     1: 'InvalidHandle - The FTDI functions have not been initialized.',
#     2: 'DeviceNotFound - The Device could not be found.',
#     3: 'DeviceNotOpened - The Device must be opened before it can be accessed ',
#     4: 'IOError - An I/O Error has occured in the FTDI chip.',
#     5: 'InsufficientResources - There are Insufficient resources to run this application.',
#     6: 'InvalidParameter - An invalid parameter has been supplied to the device.',
#     7: 'DeviceNotPresent - The Device is no longer present',
#     8: 'IncorrectDevice - The device detected does not match that expected./term>',
#     32: 'ALREADY_OPEN - Attempt to open a device that was already open.',
#     33: 'NO_RESPONSE - The device has stopped responding.',
#     34: 'NOT_IMPLEMENTED - This function has not been implemented.',
#     35: 'FAULT_REPORTED - The device has reported a fault.',
#     36: 'INVALID_OPERATION - The function could not be completed at this time.',
#     40: 'DISCONNECTING - The function could not be completed because the device is disconnected.',
#     41: 'FIRMWARE_BUG - The firmware has thrown an error.',
#     42: 'INITIALIZATION_FAILURE - The device has failed to initialize',
#     43: 'INVALID_CHANNEL - An Invalid channel address was supplied.',
#     37: 'UNHOMED - The device cannot perform this function until it has been Homed.',
#     38: ('INVALID_POSITION - The function cannot be performed as it would result in an illegal '
#          'position.'),
#     39: 'INVALID_VELOCITY_PARAMETER - An invalid velocity parameter was supplied',
#     44: 'CANNOT_HOME_DEVICE - This device does not support Homing ',
#     45: 'TL_JOG_CONTINOUS_MODE - An invalid jog mode was supplied for the jog function.'
# }

        # _sigs_ = {name: all_sigs[name] for name in [
        #     # 'Open',
        #     # 'Close',
        #     # 'Identify',
        #     # 'GetHardwareInfo',
        #     # 'GetFirmwareVersion',
        #     # 'GetSoftwareVersion',
        #     # 'LoadSettings',
        #     # 'PersistSettings',
        #     # 'GetNumberPositions',
        #     # 'CanHome',
        #     # 'Home',
        #     # 'NeedsHoming',
        #     # 'MoveToPosition',
        #     # 'GetPosition',
        #     # 'GetPositionCounter',
        #     # 'RequestStatus',
        #     # 'RequestStatusBits',
        #     # 'GetStatusBits',
        #     # 'StartPolling',
        #     # 'PollingDuration',
        #     # 'StopPolling',
        #     # 'RequestSettings',
        #     # 'ClearMessageQueue',
        #     # 'RegisterMessageCallback',
        #     # 'MessageQueueSize',
        #     # 'GetNextMessage',
        #     # 'WaitForMessage',
        #     # 'GetMotorParamsExt',
        #     # 'SetJogStepSize',
        #     # 'GetJogVelParams',
        #     # 'GetBacklash',
        #     # 'SetBacklash',
        #     # 'GetLimitSwitchParams',
        #     # 'GetLimitSwitchParamsBlock',
        #     'Open',
        #     'Close',
        #     'Identify',
        #     'GetLEDswitches',
        #     'SetLEDswitches',
        #     'GetHardwareInfo',
        #     'GetHardwareInfoBlock',
        #     'GetHubBay',
        #     'GetSoftwareVersion',
        #     'LoadSettings',
        #     'PersistSettings',
        #     'DisableChannel',
        #     'EnableChannel',
        #     'GetNumberPositions',
        #     'CanHome',
        #     'NeedsHoming',
        #     'Home',
        #     'MoveToPosition',
        #     'GetPosition',
        #     'GetHomingVelocity',
        #     'SetHomingVelocity',
        #     'MoveRelative',
        #     'GetJogMode',
        #     'SetJogMode',
        #     'SetJogStepSize',
        #     'GetJogStepSize',
        #     'GetJogVelParams',
        #     'SetJogVelParams',
        #     'MoveJog',
        #     'SetVelParams',
        #     'GetVelParams',
        #     'MoveAtVelocity',
        #     'SetDirection',
        #     'StopImmediate',
        #     'StopProfiled',
        #     'GetBacklash',
        #     'SetBacklash',
        #     'GetPositionCounter',
        #     'SetPositionCounter',
        #     'GetEncoderCounter',
        #     'SetEncoderCounter',
        #     'GetLimitSwitchParams',
        #     'SetLimitSwitchParams',
        #     'GetSoftLimitMode',
        #     'SetLimitsSoftwareApproachPolicy',
        #     'GetButtonParams',
        #     'SetButtonParams',
        #     'SetPotentiometerParams',
        #     'GetPotentiometerParams',
        #     'GetVelParamsBlock',
        #     'SetVelParamsBlock',
        #     'SetMoveAbsolutePosition',
        #     'GetMoveAbsolutePosition',
        #     'MoveAbsolute',
        #     'SetMoveRelativeDistance',
        #     'GetMoveRelativeDistance',
        #     'MoveRelativeDistance',
        #     'GetHomingParamsBlock',
        #     'SetHomingParamsBlock',
        #     'GetJogParamsBlock',
        #     'SetJogParamsBlock',
        #     'GetButtonParamsBlock',
        #     'SetButtonParamsBlock',
        #     'GetPotentiometerParamsBlock',
        #     'SetPotentiometerParamsBlock',
        #     'GetLimitSwitchParamsBlock',
        #     'SetLimitSwitchParamsBlock',
        #     'GetDCPIDParams',
        #     'SetDCPIDParams',
        #     'SuspendMoveMessages',
        #     'ResumeMoveMessages',
        #     'RequestPosition',
        #     'RequestStatusBits',
        #     'GetStatusBits',
        #     'StartPolling',
        #     'PollingDuration',
        #     'StopPolling',
        #     'RequestSettings',
        #     'GetStageAxisMinPos',
        #     'GetStageAxisMaxPos',
        #     'SetStageAxisLimits',
        #     'SetMotorTravelMode',
        #     'GetMotorTravelMode',
        #     'SetMotorParams',
        #     'GetMotorParams',
        #     'SetMotorParamsExt',
        #     'GetMotorParamsExt',
        #     'ClearMessageQueue',
        #     'RegisterMessageCallback',
        #     'MessageQueueSize',
        #     'GetNextMessage',
        #     'WaitForMessage',
        # ]}


# class NiceTDC001(NiceLib):
#     """Mid-level wrapper for Thorlabs.MotionControl.TCube.DCServo.dll"""
#     _ffi_ = ffi
#     _ffilib_ = lib
#     _prefix_ = ('CC_', 'TLI_')
#     _buflen_ = 512
#     _ret_ = ret_errcheck
#
#     BuildDeviceList = Sig()
#     GetDeviceListSize = Sig(ret=ret_return)
#     GetDeviceInfo = Sig('in', 'out', ret=ret_success)
#     GetDeviceList = Sig('out')
#     GetDeviceListByType = Sig('out', 'in')
#     GetDeviceListByTypes = Sig('out', 'in', 'in')
#     GetDeviceListExt = Sig('buf', 'len')
#     GetDeviceListByTypeExt = Sig('buf', 'len', 'in')
#     GetDeviceListByTypesExt = Sig('buf', 'len', 'in', 'in')
#     InitializeSimulations = Sig(ret=ret_ignore)
#     UninitializeSimulations = Sig(ret=ret_ignore)
#
#     class Device(NiceObject):
#         Open = Sig('in')
#         Close = Sig('in', ret=ret_return)
#         Identify = Sig('in', ret=ret_return)
#         GetLEDswitches = Sig('in', ret=ret_return)
#         SetLEDswitches = Sig('in', 'in')
#         GetHardwareInfo = Sig('in', 'buf', 'len', 'out', 'out', 'buf', 'len', 'out', 'out', 'out')
#         GetHardwareInfoBlock = Sig('in', 'out')
#         GetHubBay = Sig('in', ret=ret_return)
#         GetSoftwareVersion = Sig('in', ret=ret_return)
#         LoadSettings = Sig('in', ret=ret_success)
#         PersistSettings = Sig('in', ret=ret_success)
#         DisableChannel = Sig('in')
#         EnableChannel = Sig('in')
#         GetNumberPositions = Sig('in', ret=ret_return)
#         CanHome = Sig('in', ret=ret_success)
#         NeedsHoming = Sig('in', ret=ret_return)
#         Home = Sig('in')
#         MoveToPosition = Sig('in', 'in')
#         GetPosition = Sig('in', ret=ret_return)
#         GetHomingVelocity = Sig('in', ret=ret_return)
#         SetHomingVelocity = Sig('in', 'in')
#         MoveRelative = Sig('in', 'in')
#         GetJogMode = Sig('in', 'out', 'out')
#         SetJogMode = Sig('in', 'in', 'in')
#         SetJogStepSize = Sig('in', 'in')
#         GetJogStepSize = Sig('in', ret=ret_return)
#         GetJogVelParams = Sig('in', 'out', 'out')
#         SetJogVelParams = Sig('in', 'in', 'in')
#         MoveJog = Sig('in', 'in')
#         SetVelParams = Sig('in', 'in', 'in')
#         GetVelParams = Sig('in', 'out', 'out')
#         MoveAtVelocity = Sig('in', 'in')
#         SetDirection = Sig('in', 'in', ret=ret_return)
#         StopImmediate = Sig('in')
#         StopProfiled = Sig('in')
#         GetBacklash = Sig('in', ret=ret_return)
#         SetBacklash = Sig('in', 'in')
#         GetPositionCounter = Sig('in', ret=ret_return)
#         SetPositionCounter = Sig('in', 'in')
#         GetEncoderCounter = Sig('in', ret=ret_return)
#         SetEncoderCounter = Sig('in', 'in')
#         GetLimitSwitchParams = Sig('in', 'out', 'out', 'out', 'out', 'out')
#         SetLimitSwitchParams = Sig('in', 'in', 'in', 'in', 'in', 'in')
#         GetSoftLimitMode = Sig('in', ret=ret_return)
#         SetLimitsSoftwareApproachPolicy = Sig('in', 'in', ret=ret_return)
#         GetButtonParams = Sig('in', 'out', 'out', 'out', 'out')
#         SetButtonParams = Sig('in', 'in', 'in', 'in')
#         SetPotentiometerParams = Sig('in', 'in', 'in', 'in')
#         GetPotentiometerParams = Sig('in', 'in', 'out', 'out')
#         GetVelParamsBlock = Sig('in', 'out')
#         SetVelParamsBlock = Sig('in', 'in')
#         SetMoveAbsolutePosition = Sig('in', 'in')
#         GetMoveAbsolutePosition = Sig('in', ret=ret_return)
#         MoveAbsolute = Sig('in')
#         SetMoveRelativeDistance = Sig('in', 'in')
#         GetMoveRelativeDistance = Sig('in', ret=ret_return)
#         MoveRelativeDistance = Sig('in')
#         GetHomingParamsBlock = Sig('in', 'out')
#         SetHomingParamsBlock = Sig('in', 'in')
#         GetJogParamsBlock = Sig('in', 'out')
#         SetJogParamsBlock = Sig('in', 'in')
#         GetButtonParamsBlock = Sig('in', 'out')
#         SetButtonParamsBlock = Sig('in', 'in')
#         GetPotentiometerParamsBlock = Sig('in', 'out')
#         SetPotentiometerParamsBlock = Sig('in', 'in')
#         GetLimitSwitchParamsBlock = Sig('in', 'out')
#         SetLimitSwitchParamsBlock = Sig('in', 'in')
#         GetDCPIDParams = Sig('in', 'out')
#         SetDCPIDParams = Sig('in', 'in')
#         SuspendMoveMessages = Sig('in')
#         ResumeMoveMessages = Sig('in')
#         RequestPosition = Sig('in')
#         RequestStatusBits = Sig('in', ret=ret_return)
#         GetStatusBits = Sig('in', ret=ret_return)
#         StartPolling = Sig('in', 'in', ret=ret_success)
#         PollingDuration = Sig('in', ret=ret_return)
#         StopPolling = Sig('in', ret=ret_return)
#         RequestSettings = Sig('in')
#         GetStageAxisMinPos = Sig('in', ret=ret_return)
#         GetStageAxisMaxPos = Sig('in', ret=ret_return)
#         SetStageAxisLimits = Sig('in', 'in', 'in')
#         SetMotorTravelMode = Sig('in', 'in')
#         GetMotorTravelMode = Sig('in', ret=ret_return)
#         SetMotorParams = Sig('in', 'in', 'in', 'in')
#         GetMotorParams = Sig('in', 'out', 'out', 'out')
#         SetMotorParamsExt = Sig('in', 'in', 'in', 'in')
#         GetMotorParamsExt = Sig('in', 'out', 'out', 'out')
#         ClearMessageQueue = Sig('in', ret=ret_return)
#         RegisterMessageCallback = Sig('in', 'in', ret=ret_return)
#         MessageQueueSize = Sig('in', ret=ret_return)
#         GetNextMessage = Sig('in', 'out', 'out', 'out', ret=ret_success)
#         WaitForMessage = Sig('in', 'in', 'in', 'in', ret=ret_success)


# class KDC101Error(Error):
#     pass
#
#
# error_dict = {
#     0: 'OK - Success  ',
#     1: 'InvalidHandle - The FTDI functions have not been initialized.',
#     2: 'DeviceNotFound - The Device could not be found.',
#     3: 'DeviceNotOpened - The Device must be opened before it can be accessed ',
#     4: 'IOError - An I/O Error has occured in the FTDI chip.',
#     5: 'InsufficientResources - There are Insufficient resources to run this application.',
#     6: 'InvalidParameter - An invalid parameter has been supplied to the device.',
#     7: 'DeviceNotPresent - The Device is no longer present',
#     8: 'IncorrectDevice - The device detected does not match that expected./term>',
#     32: 'ALREADY_OPEN - Attempt to open a device that was already open.',
#     33: 'NO_RESPONSE - The device has stopped responding.',
#     34: 'NOT_IMPLEMENTED - This function has not been implemented.',
#     35: 'FAULT_REPORTED - The device has reported a fault.',
#     36: 'INVALID_OPERATION - The function could not be completed at this time.',
#     40: 'DISCONNECTING - The function could not be completed because the device is disconnected.',
#     41: 'FIRMWARE_BUG - The firmware has thrown an error.',
#     42: 'INITIALIZATION_FAILURE - The device has failed to initialize',
#     43: 'INVALID_CHANNEL - An Invalid channel address was supplied.',
#     37: 'UNHOMED - The device cannot perform this function until it has been Homed.',
#     38: ('INVALID_POSITION - The function cannot be performed as it would result in an illegal '
#          'position.'),
#     39: 'INVALID_VELOCITY_PARAMETER - An invalid velocity parameter was supplied',
#     44: 'CANNOT_HOME_DEVICE - This device does not support Homing ',
#     45: 'TL_JOG_CONTINOUS_MODE - An invalid jog mode was supplied for the jog function.'
# }
