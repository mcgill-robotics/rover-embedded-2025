"""
CAN FD command transmitter for the B-G431B-ESC1

Provides a high-level API for sending motor commands over the CAN bus
Pairs with the datalogger (which records everything) and the dashboard
(which visualises responses)

Usage:
    from esc_can.commander import CANCommander

    cmd = CANCommander(bus)                          # python-can Bus
    cmd.set_position(device_id=1, degrees=45.0)      # single position
    cmd.set_velocity(device_id=1, rad_per_sec=0.5)   # single velocity
    cmd.stop(device_id=1)                             # emergency stop
    cmd.ping(device_id=1)                             # request ping
    cmd.read(device_id=1, spec=ReadSpec.TEMPERATURE)  # read a value

All frames sent are also optionally logged to the datalogger so they
appear in the dashboard alongside ESC responses.

CAN ID encoding matches the firmware exactly:
    Bit [10]   Sender       = 0 (MASTER)
    Bit [9]    Action        = 0 (RUN) or 1 (READ)
    Bit [8]    MotorConfig   = 1 (SINGLE) or 0 (MULTIPLE)
    Bit [7]    MotorType     = 0 (DRIVE) or 1 (STEERING)
    Bits [6:4] Specification = RunSpec or ReadSpec
    Bits [3:0] DeviceID      = 0–15
"""

from __future__ import annotations

import struct
import time
from typing import Optional, TYPE_CHECKING

from .protocol import (
    Action,
    MotorConfig,
    MotorType,
    ReadSpec,
    RunSpec,
    Sender,
    encode_can_id,
    pack_single_float,
)

if TYPE_CHECKING:
    from .datalogger import CANDataLogger


class CANCommander:
    """Send CAN FD commands to ESC(s) on the bus.

    Parameters
    bus : can.BusABC or None
        A python-can Bus instance.  If None, frames are built but not
        sent (dry-run / offline mode for testing).
    logger : CANDataLogger or None
        If provided, every transmitted frame is also logged to the DB
        so the dashboard can display master-originated commands.
    motor_type : MotorType
        Default motor type for all commands.  Can be overridden per call.
    is_fd : bool
        Whether to send frames as CAN FD.  The B-G431B-ESC1 firmware
        currently accepts both classic CAN and CAN FD frames.
    """

    def __init__(
        self,
        bus=None,
        logger: Optional["CANDataLogger"] = None,
        motor_type: MotorType = MotorType.DRIVE,
        is_fd: bool = False,
    ) -> None:
        self._bus = bus
        self._logger = logger
        self._motor_type = motor_type
        self._is_fd = is_fd
        self._tx_count = 0

    
    # Run commands
    def set_position(
        self,
        device_id: int,
        degrees: float,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a position setpoint in degrees.

        The firmware receives this as a float in the CAN payload,
        converts degrees → radians via degreesToRad(), and passes
        it to the S-curve planner via buildNewCurve().

        Firmware path:
            CAN_Parse_MSG → Handle_Run_Command → case RUN_POSITION
            → positionSetpoint = degreesToRad(information)
            → newSetpointDetected = true
            → controlMode = MODE_POSITION

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.POSITION,
            value=degrees,
            motor_type=motor_type,
        )

    def set_velocity(
        self,
        device_id: int,
        rad_per_sec: float,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a velocity setpoint in rad/s.

        The firmware receives this value directly and passes it to
        velCtrlSetDemand().  If not already in velocity mode, it
        calls velCtrlStart() first and sets controlMode = MODE_VELOCITY.

        Firmware path:
            CAN_Parse_MSG → Handle_Run_Command → case RUN_SPEED
            → velCtrlSetDemand(velCtrl, information)  // value in rad/s

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.SPEED,
            value=rad_per_sec,
            motor_type=motor_type,
        )

    def stop(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a STOP command.

        Firmware path:
            CAN_Parse_MSG → Handle_Run_Command → case RUN_STOP

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.STOP,
            value=0.0,
            motor_type=motor_type,
        )

    def acknowledge_faults(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send an ACKNOWLEDGE_FAULTS command.

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.ACKNOWLEDGE_FAULTS,
            value=0.0,
            motor_type=motor_type,
        )

    def calibrate(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a CALIBRATION command.

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.CALIBRATION,
            value=0.0,
            motor_type=motor_type,
        )

    
    # Read requests
    def read(
        self,
        device_id: int,
        spec: ReadSpec,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a read request.  The ESC will respond with the value.

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_read(
            device_id=device_id,
            read_spec=spec,
            motor_type=motor_type,
        )

    def ping(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a PING read request.  ESC responds with 69.0f.

        Returns the 11-bit arb_id that was sent.
        """
        return self.read(device_id=device_id, spec=ReadSpec.PING,
                         motor_type=motor_type)

    def read_position(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Request the current position from an ESC."""
        return self.read(device_id=device_id, spec=ReadSpec.POSITION,
                         motor_type=motor_type)

    def read_speed(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Request the current speed from an ESC."""
        return self.read(device_id=device_id, spec=ReadSpec.SPEED,
                         motor_type=motor_type)

    def read_voltage(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Request bus voltage from an ESC."""
        return self.read(device_id=device_id, spec=ReadSpec.VOLTAGE,
                         motor_type=motor_type)

    def read_temperature(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Request temperature from an ESC."""
        return self.read(device_id=device_id, spec=ReadSpec.TEMPERATURE,
                         motor_type=motor_type)

    
    # Properties
    @property
    def tx_count(self) -> int:
        """Total number of frames transmitted."""
        return self._tx_count

    @property
    def bus_connected(self) -> bool:
        """True if a real bus is attached."""
        return self._bus is not None

    
    # Internals
    def _send_run(
        self,
        device_id: int,
        run_spec: RunSpec,
        value: float,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Build and send a RUN command frame."""
        mt = motor_type if motor_type is not None else self._motor_type

        arb_id = encode_can_id(
            sender=Sender.MASTER,
            action=Action.RUN,
            motor_config=MotorConfig.SINGLE,
            motor_type=mt,
            spec=int(run_spec),
            device_id=device_id,
        )

        # Payload: 4-byte little-endian float + 4 bytes zero padding
        # Matches SingleExtractFloatFromCAN() on firmware side
        data = pack_single_float(value)

        self._transmit(arb_id, data)
        return arb_id

    def _send_read(
        self,
        device_id: int,
        read_spec: ReadSpec,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Build and send a READ request frame."""
        mt = motor_type if motor_type is not None else self._motor_type

        arb_id = encode_can_id(
            sender=Sender.MASTER,
            action=Action.READ,
            motor_config=MotorConfig.SINGLE,
            motor_type=mt,
            spec=int(read_spec),
            device_id=device_id,
        )

        # Read requests carry no payload data
        data = b"\x00" * 8

        self._transmit(arb_id, data)
        return arb_id

    def _transmit(self, arb_id: int, data: bytes) -> None:
        """Send frame on the bus and optionally log it."""
        if self._bus is not None:
            try:
                import can
                msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=False,
                    is_fd=self._is_fd,
                    bitrate_switch=self._is_fd,
                    dlc=len(data),
                )
                self._bus.send(msg)
            except Exception as e:
                # Don't crash the caller — log the error
                import sys
                print(f"[commander] TX error: {e}", file=sys.stderr)

        # Log the transmitted frame if a logger is attached
        if self._logger is not None:
            self._logger.log_raw(
                arb_id=arb_id,
                data=data,
                is_fd=self._is_fd,
                brs=self._is_fd,
                is_rx=False,
            )

        self._tx_count += 1



# Convenience: build a commander from CLI args (used by dashboard)
def open_commander(
    port: Optional[str] = None,
    interface: str = "slcan",
    bitrate: int = 500_000,
    data_bitrate: int = 2_000_000,
    logger: Optional["CANDataLogger"] = None,
    motor_type: MotorType = MotorType.DRIVE,
) -> CANCommander:
    """Create a CANCommander with a live bus connection.

    If ``port`` is None, returns a dry-run commander (no bus).
    """
    bus = None
    if port:
        try:
            import can
            bus = can.Bus(
                interface=interface,
                channel=port,
                bitrate=bitrate,
                data_bitrate=data_bitrate,
                fd=True,
            )
        except Exception as e:
            import sys
            print(f"[commander] Could not open bus on {port}: {e}",
                  file=sys.stderr)

    return CANCommander(bus=bus, logger=logger, motor_type=motor_type)