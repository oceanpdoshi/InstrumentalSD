from __future__ import division
from enum import Enum
from time import sleep
from .... import u, Q_
from ....log import get_logger
from ... import Facet
from ....errors import Error
from ...util import check_units, check_enums
from .. import Motion
from .kdc_midlib import NiceKDC
from .common import (KinesisError, MessageType, GenericDevice, GenericMotor,
                     GenericDCMotor, MessageIDs)
import nicelib  # noqa (nicelib dep is hidden behind import of isc_midlib)

log = get_logger(__name__)

STATUS_MOVING_CW = 0x10
STATUS_MOVING_CCW = 0x20
STATUS_JOGGING_CW = 0x40
STATUS_JOGGING_CCW = 0x80


### code copied from tdc_001.py

class TravelMode(Enum):
    Linear = 1
    Rotational = 2


class SoftwareApproachPolicy(Enum):
    DisableOutsideRange = 0
    DisableFarOutsideRange = 1
    TruncateBeyondLimit = 2
    AllowAll = 3


class KDC101(Motion):
    """ Controlling Thorlabs KDC101 K-Cube DC Servo Motor Controllers

    The polling period, which is how often the device updates its status, is
    passed as a pint pint quantity with units of time and is optional argument,
    with a default of 200ms
    """
    _INST_PARAMS_ = ['serial']
    _lib = NiceKDC
    # Enums
    MessageType = MessageType
    GenericDevice = GenericDevice
    GenericMotor = GenericMotor
    GenericDCMotor = GenericDCMotor

    @check_units(polling_period='ms')
    def _initialize(self, polling_period='200ms', allow_all_moves=True):
        """
        Parameters
        ----------
        polling_period : pint Quantity with units of time
        """
        self.SoftwareApproachPolicy = SoftwareApproachPolicy
        self.TravelMode = TravelMode
        self.serial = self._paramset['serial']

        self._open()
        self._start_polling(polling_period)
        # self._wait_for_message(GenericDevice.SettingsInitialized)
        try:
            self.dev.LoadSettings()
            # self._set_real_world_units()
        except Error:
            warn_string = "KDC101 with SN {} did not initialize successfully."
            warn_string += "  The motion control device connected to the controller "
            warn_string += "may not support auto-loading of parameters. \n\n"
            warn(warn_string.format(self.serial) + traceback.format_exc())
        if allow_all_moves:
            self._set_software_approach_policy(self.SoftwareApproachPolicy.AllowAll)
            # stage_max_pos = 2**30
            # self.dev.SetStageAxisLimits(-stage_max_pos, stage_max_pos)
            # assert self.dev.GetStageAxisMaxPos() == stage_max_pos
            # assert self.dev.GetStageAxisMinPos() == -stage_max_pos
        travel_mode = self.TravelMode(self.dev.GetMotorTravelMode())
        if travel_mode == self.TravelMode.Linear:
            self.units = u('mm')
        if travel_mode == self.TravelMode.Rotational:
            self.units = u('degrees')
        self.offset = self._paramset.get('offset', Q_(0.0,self.units))
        self.serial = self._paramset['serial']
        steps_per_rev, gearbox_ratio, pitch = self.get_motor_params()
        self._unit_scaling = 1/Q_(pitch/steps_per_rev/float(gearbox_ratio), self.units)

        # self._unit_scaling = (gear_box_ratio * micro_steps_per_step *
        #                       steps_per_rev / (360.0 * u.deg))
        # self.dev = NiceKDC.Device(self.serial)


    def _open(self):
        NiceKDC.BuildDeviceList()  # Necessary?
        self.dev = NiceKDC.Device(self.serial)
        self.dev.Open()

    def close(self):
        self.dev.StopPolling()
        self.dev.Close()

    @check_units(polling_period='ms')
    def _start_polling(self, polling_period='200ms'):
        """Starts polling the device to update its status with the given period provided, rounded
        to the nearest millisecond

        Parameters
        ----------
        polling_period: pint quantity with units of time
        """
        self.polling_period = polling_period
        self.dev.StartPolling(self.polling_period.m_as('ms'))

    # def _set_real_world_unit_conversion(self):
    #     steps_per_rev, gearbox_ratio, pitch = self.get_motor_params()
    #     cf = Q_(pitch/steps_per_rev/float(gearbox_ratio), self.real_world_units)
    #     self.encoder_to_real_world_units_conversion_factor = cf
    #
    # def _set_real_world_units(self):
    #     travel_mode = self.TravelMode(self.dev.GetMotorTravelMode())
    #     if travel_mode == self.TravelMode.Linear:
    #         real_world_units = u('mm')
    #     if travel_mode == self.TravelMode.Rotational:
    #         real_world_units = u('degrees')
    #     self.real_world_units = real_world_units
    #
    #     self._set_real_world_unit_conversion()

    def get_status(self):
        """ Returns the status registry bits from the device."""
        self.dev.RequestStatusBits()
        status_bits = self.dev.GetStatusBits()
        return self.Status(status_bits)

    def set_motor_params(self, steps_per_rev, gearbox_ratio, pitch):
        """ Sets the motor stage parameters.

        Parameters
        ----------
        steps_per_rev: int
        gearbox_ratio: int
        pitch: float """
        self.pitch = pitch
        self.gearbox_ratio = gearbox_ratio
        self.steps_per_rev = steps_per_rev

        value = self.dev.SetMotorParams(steps_per_rev, gearbox_ratio, pitch)
        return value

    def get_motor_params(self):
        """ Gets the stage motor parameters.

        Returns
        -------
        (steps_per_rev: int,
        gearbox_ratio: int,
        pitch: float)"""
        steps_per_rev, gearbox_ratio, pitch = self.dev.GetMotorParams()
        return int(steps_per_rev), int(gearbox_ratio), float(pitch)

    @check_units(angle='deg')
    def move_to(self, angle, wait=False):
        """Rotate the stage to the given angle

        Parameters
        ----------
        angle : Quantity
            Angle that the stage will rotate to. Takes the stage offset into account.
        """
        log.debug("Moving stage to {}".format(angle))
        log.debug("Current position is {}".format(self.position))
        self.dev.ClearMessageQueue()
        self.dev.MoveToPosition(self._to_dev_units(angle + self.offset))
        if wait:
            self.wait_for_move()

    def _decode_message(self, msg_tup):
        msg_type_int, msg_id_int, msg_data_int = msg_tup
        msg_type = MessageType(msg_type_int)
        msg_id = MessageIDs[msg_type](msg_id_int)
        return (msg_id, msg_data_int)

    def _wait_for_message(self, match_id):
        if not isinstance(match_id, (GenericDevice, GenericMotor, GenericDCMotor)):
            raise ValueError("Must specify message ID via enum")

        msg_id, msg_data = self._decode_message(self.dev.WaitForMessage())
        log.debug("Received kinesis message ({}: {})".format(msg_id, msg_data))
        while msg_id is not match_id:
            msg_id, msg_data = self._decode_message(self.dev.WaitForMessage())
            log.debug("Received kinesis message ({}: {})".format(msg_id, msg_data))

    def _check_for_message(self, match_id):
        """Check if a message of the given type and id is in the queue"""
        if not isinstance(match_id, (GenericDevice, GenericMotor, GenericDCMotor)):
            raise ValueError("Must specify message ID via enum")

        while True:
            try:
                msg_id, msg_data = self._decode_message(self.dev.GetNextMessage())
            except KinesisError:
                return False

            log.debug("Received kinesis message ({}: {})".format(msg_id, msg_data))
            if msg_id is match_id:
                return True

    def wait_for_move(self):
        """Wait for the most recent move to complete"""
        self._wait_for_message(GenericMotor.Moved)

    def move_finished(self):
        """Check if the most recent move has finished"""
        return self._check_for_message(GenericMotor.Moved)

    def _to_real_units(self, dev_units):
        return (dev_units / self._unit_scaling).to('deg')

    @check_units(real_units='deg')
    def _to_dev_units(self, real_units):
        return int(round(float(real_units * self._unit_scaling)))

    def home(self, wait=False):
        """Home the stage

        Parameters
        ----------
        wait : bool, optional
            Wait until the stage has finished homing to return
        """
        self.dev.ClearMessageQueue()
        self.dev.Home()

        if wait:
            self.wait_for_home()

    def wait_for_home(self):
        """Wait for the most recent homing operation to complete"""
        self._wait_for_message(GenericMotor.Homed)

    def homing_finished(self):
        """Check if the most recent homing operation has finished"""
        return self._check_for_message(GenericMotor.Homed)

    @Facet
    def needs_homing(self):
        """True if the device needs to be homed before a move can be performed"""
        return bool(self.dev.NeedsHoming())

    @Facet(units='deg')
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, offset):
        self._offset = offset

    @Facet(units='deg')
    def position(self):
        return self._to_real_units(self.dev.GetPosition()) - self.offset

    @Facet
    def is_homing(self):
        return bool(self.dev.GetStatusBits() & 0x00000200)

    @Facet
    def is_moving(self):
        return bool(self.dev.GetStatusBits() & 0x00000030)

    def get_next_message(self):
        msg_type, msg_id, msg_data = self.dev.GetNextMessage()
        type = MessageType(msg_type)
        id = MessageIDs[type](msg_id)
        return (type, id, msg_data)

    def get_messages(self):
        messages = []
        while True:
            try:
                messages.append(self.get_next_message())
            except Exception:
                break
        return messages

    @check_enums(software_approach_policy=SoftwareApproachPolicy)
    def _set_software_approach_policy(self, software_approach_policy):
        """ Controls what range of values the motor is allowed to move to.

        This is done using the SoftwareApproachPolicy enumerator """
        self.dev.SetLimitsSoftwareApproachPolicy(software_approach_policy.value)

    def _get_software_approach_policy(self):
        """ Gets the range of values the motor is allowed to move to.

        Returns an instance of SoftwareApproachPolicy enumerator """
        software_approach_policy = self.dev.GetSoftLimitMode()
        return self.SoftwareApproachPolicy(software_approach_policy)

    class Status():
        """ Stores information about the status of the device from the status
        bits. """
        def __init__(self, status_bits):
            nbits = 32
            self._status_bits = status_bits
            self._bits = zeros(nbits)
            for i in range(nbits):
                self._bits[i] = (status_bits%2**(i+1))/2**i
            self.CW_LimSwitch_Contact = bool(self._bits[0])
            self.CCW_LimSwitch_Contact = bool(self._bits[1])
            self.MotorMovingCW = bool(self._bits[4])
            self.MotorMovingCCW = bool(self._bits[5])
            self.MotorJoggingCW = bool(self._bits[6])
            self.MotorJoggingCCW = bool(self._bits[7])
            self.MotorHoming = bool(self._bits[9])
            self.MotorHomed = bool(self._bits[10])
            self.isActive = bool(self._bits[29])
            self.isEnabled = bool(self._bits[31])

            temp = self.MotorHoming or self.MotorJoggingCCW or self.MotorJoggingCW
            self.isMoving = temp or self.MotorMovingCCW or self.MotorMovingCW


    # def get_position(self):
    #     """ Returns the position of the motor.
    #
    #     Note that this represents the position at the most recent polling
    #     event."""
    #     self.dev.RequestPosition()
    #     position = self.dev.GetPosition()
    #     return position*self.encoder_to_real_world_units_conversion_factor
    #
    # def move_to(self, position):
    #     """ Moves to the indicated position
    #
    #
    #     Returns immediately.
    #
    #     Parameters
    #     ----------
    #     position: pint quantity of units self.real_world_units """
    #     position = Q_(position)
    #     position = self._get_encoder_value(position)
    #     value = self.dev.MoveToPosition(position)
    #     return value
    #
    # def _get_encoder_value(self, position):
    #     position = position.to(self.real_world_units)
    #     position = position/self.encoder_to_real_world_units_conversion_factor
    #     return int(position.magnitude)
    #
    # @check_units(delay='ms')
    # def move_and_wait(self, position, delay='100ms', tol=1):
    #     """ Moves to the indicated position and waits until that position is
    #     reached.
    #
    #     Parameters
    #     ----------
    #     position: pint quantity of units self.real_world_units
    #     delay: pint quantity with units of time
    #         the period with which the position of the motor is checked.
    #     tol: int
    #         the tolerance, in encoder units, to which the motor is considered
    #         at position"""
    #     position = Q_(position)
    #     self.move_to(position)
    #     while not self.at_position(position, tol):
    #         # sleep(delay.to('s').magnitude)
    #         sleep(self.polling_period.to('s').magnitude)
    #
    # def at_position(self, position, tol=1):
    #     """Indicates whether the motor is at the given position.
    #
    #
    #
    #     Parameters
    #     ----------
    #     position: pint quantity of units self.real_world_units
    #     tol: int representing the number of encoder counts within which the
    #     motor is considered to be at position"""
    #     position = Q_(position)
    #     self.dev.RequestPosition()
    #     enc_position = self._get_encoder_value(position)
    #     at_pos = abs(self.dev.GetPosition() - enc_position) <= tol
    #     return at_pos

    # def home(self):
    #     """ Homes the device """
    #     return self.dev.Home()
