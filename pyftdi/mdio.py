# Copyright (c) masw <masw@masw.tech>
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""MDIO support for PyFdti"""

#pylint: disable-msg=too-many-lines
#pylint: disable-msg=too-many-locals
#pylint: disable-msg=too-many-instance-attributes
#pylint: disable-msg=too-many-public-methods
#pylint: disable-msg=too-many-arguments
#pylint: disable-msg=too-many-branches
#pylint: disable-msg=too-many-statements


from binascii import hexlify
from collections import namedtuple
from logging import getLogger
from struct import calcsize as scalc, pack as spack, unpack as sunpack
from threading import Lock
from typing import Any, Iterable, Mapping, Optional, Tuple, Union
from usb.core import Device as UsbDevice
from ftdi import Ftdi, FtdiFeatureError
from misc import to_bool


class MdioIOError(IOError):
    """Mdio I/O error"""


class MdioNackError(MdioIOError):
    """Mdio NACK receive from slave"""


class MdioTimeoutError(TimeoutError):
    """Mdio timeout on polling"""


class MdioPort:
    """MDIO port.

       An MDIO port is never instanciated directly:
       use :py:meth:`MdioController.get_port()` method to obtain an MDIO port.


       Example:

       >>> ctrl = MdioController()
       >>> ctrl.configure('ftdi://ftdi:232h/1')
       >>> mdio = ctrl.get_port(0x0)
       >>> mdio.write_to(0x10001, [0x12, 0x34])
    """
    FORMATS = {scalc(fmt): fmt for fmt in 'BHI'}

    def __init__(self, controller: 'MdioController', address: int):
        self._controller = controller
        self._address = address
        self._endian = '<'
        self._format = 'B'

    def configure_register(self,
                           bigendian: bool = False, width: int = 1) -> None:
        """Reconfigure the format of the slave address register (if any)

            :param bigendian: True for a big endian encoding, False otherwise
            :param width: width, in bytes, of the register
        """
        try:
            self._format = self.FORMATS[width]
        except KeyError as exc:
            raise MdioIOError('Unsupported integer width') from exc
        self._endian = '>' if bigendian else '<'

    def read_from(self, regaddr: int, readlen: int = 0,
                   accessByClass22: bool = True) -> bytes:
        """Read one or more bytes from a remote slave

           :param regaddr: slave register address to read from
           :param readlen: count of bytes to read out.
           :param accessByClass22: whether to emit a accessByClass22 sequence (w/ address)
           :return: data read out from the slave
           :raise MdioIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.exchange(
            self._address,
            out=self._make_buffer(regaddr), readlen=readlen)

    def write_to(self, regaddr: int,
                 out: Union[bytes, bytearray, Iterable[int]],
                 accessByClass22: bool = True):
        """Read one or more bytes from a remote slave

           :param regaddr: slave register address to write to
           :param out: the byte buffer to send
           :param accessByClass22: whether to emit a accessByClass22 sequence (w/ address)
           :raise MdioIOError: if device is not configured or input parameters
                              are invalid
        """
        return self._controller.write(
            self._address,
            out=self._make_buffer(regaddr, out))

    def exchange(self, out: Union[bytes, bytearray, Iterable[int]] = b'',
                 readlen: int = 0,
                 accessByClass22: bool = True) -> bytes:
        """Perform an exchange or a transaction with the Mdio slave

           :param out: an array of bytes to send to the Mdio slave,
                       may be empty to only read out data from the slave
           :param readlen: count of bytes to read out from the slave,
                       may be zero to only write to the slave
           :param accessByClass22: whether to emit a accessByClass22 sequence (w/ address)
           :return: data read out from the slave
        """
        return self._controller.exchange(
            self._address,
            readlen)


    def flush(self) -> None:
        """Force the flush of the HW FIFOs.
        """
        self._controller.flush()

    @property
    def frequency(self) -> float:
        """Provide the current Mdio bus frequency.
        """
        return self._controller.frequency

    @property
    def address(self) -> int:
        """Return the slave address."""
        return self._address

    def _make_buffer(self, regaddr: int,
                     out: Union[bytes, bytearray, Iterable[int], None] = None)\
                     -> bytes:
        data = bytearray()
        data.extend(spack('%s%s' % (self._endian, self._format), regaddr))
        if out:
            data.extend(out)
        return bytes(data)






class MdioController:
    """Mdio master.

       An Mdio master should be instanciated only once for each FTDI port that
       supports MPSSE (one or two ports, depending on the FTDI device).

       Once configured, :py:func:`get_port` should be invoked to obtain an Mdio
       port for each Mdio slave to drive. Mdio port should handle all I/O
       requests for its associated HW slave.

       It is not recommended to use MdioController :py:func:`read`,
       :py:func:`write` or :py:func:`exchange` directly.

       * ``SCK`` should be connected to ``A*BUS0``, and ``A*BUS7`` if clock
         stretching mode is enabled
       * ``SDA`` should be connected to ``A*BUS1`` **and** ``A*BUS2``
    """

    LOW                = 0x00
    HIGH               = 0xff
    BIT0               = 0x01
    IDLE               = HIGH
    MDC_BIT            = 0x01    # AD0
    MDIO_O_BIT         = 0x02    # AD1
    MDIO_I_BIT         = 0x04    # AD2
    PAYLOAD_MAX_LENGTH = 0xFF00  # 16 bits max (- spare for control)
    MDIO_SLAVE_ADDRESS = 0x00
    MDIO_MASK          = MDC_BIT | MDIO_O_BIT | MDIO_I_BIT
    MDIO_DIR           = MDC_BIT | MDIO_O_BIT #when 1: means output, else inport


    def __init__(self):
        self._ftdi          = Ftdi()
        self._lock          = Lock()
        self.log            = getLogger('pyftdi.mdio')
        self._gpio_port     = None
        self._gpio_dir      = 0
        self._gpio_low      = 0
        self._gpio_mask     = 0
        self._mdio_mask     = 0
        self._wide_port     = False
        self._slaves        = {}
        self._frequency     = 0.0
        self._immediate     = (Ftdi.SEND_IMMEDIATE,)
        self._read_bit      = (Ftdi.READ_BITS_PVE_MSB  , 0)
        self._read_byte     = (Ftdi.READ_BYTES_PVE_MSB , 0, 0)
        self._write_byte    = (Ftdi.WRITE_BYTES_NVE_MSB, 0, 0)
        self._nack          = (Ftdi.WRITE_BITS_NVE_MSB , 0, self.HIGH)
        self._ack           = (Ftdi.WRITE_BITS_NVE_MSB , 0, self.LOW)
        self._ck_delay      = 1
        self._fake_tristate = False
        self._tx_size       = 1
        self._rx_size       = 1
        self._ck_hd_sta     = 0
        self._ck_su_sto     = 0
        self._ck_idle       = 0
        self._read_optim    = True


    def configure(self, url: Union[str, UsbDevice],
                  **kwargs: Mapping[str, Any]) -> None:
        """Configure the FTDI interface as a Mdio master.

           :param url: FTDI URL string, such as ``ftdi://ftdi:232h/1``
           :param kwargs: options to configure the MDIO bus

           Accepted options:

           * ``interface``: when URL is specifed as a USB device, the interface
             named argument can be used to select a specific port of the FTDI
             device, as an integer accessByClass22ing from 1.
           * ``direction`` a bitfield specifying the FTDI GPIO direction,
             where high level defines an output, and low level defines an
             input. Only useful to setup default IOs at accessByClass22 up, use
             :py:class:`MdioGpioPort` to drive GPIOs. Note that pins reserved
             for MDIO feature take precedence over any this setting.
           * ``initial`` a bitfield specifying the initial output value. Only
             useful to setup default IOs at accessByClass22 up, use
             :py:class:`MdioGpioPort` to drive GPIOs.
           * ``frequency`` float value the MDIO bus frequency in Hz
           * ``clockstretching`` boolean value to enable clockstreching.
             xD7 (GPIO7) pin should be connected back to xD0 (SCK)
           * ``debug`` to increase log verbosity, using MPSSE tracer
        """
        if 'frequency' in kwargs:
            frequency = kwargs['frequency']
            del kwargs['frequency']
        else:
            frequency = 2500000.0
        if 'interface' in kwargs:
            if isinstance(url, str):
                raise MdioIOError('url and interface are mutually exclusive')
            interface = int(kwargs['interface'])
            del kwargs['interface']
        else:
            interface = 1
        if 'rdoptim' in kwargs:
            self._read_optim = to_bool(kwargs['rdoptim'])
            del kwargs['rdoptim']
        with self._lock:
            self._mdio_mask = self.MDIO_MASK
            # until the device is open, there is no way to tell if it has a
            # wide (16) or narrow port (8). Lower API can deal with any, so
            # delay any truncation till the device is actually open
            self._set_gpio_direction(16, 0, 0)
            # as 3-phase clock frequency mode is required for MDIO mode, the
            # FTDI clock should be adapted to match the required frequency.
            kwargs['direction'] = self.MDIO_DIR | self._gpio_dir
            kwargs['initial'] = self.IDLE 
            kwargs['frequency'] = (3.0*frequency)/2.0
            if not isinstance(url, str):
                frequency = self._ftdi.open_mpsse_from_device(
                    url, interface=interface, **kwargs)
            else:
                frequency = self._ftdi.open_mpsse_from_url(url, **kwargs)
            self._frequency = (2.0*frequency)/3.0
            self._tx_size, self._rx_size = self._ftdi.fifo_sizes
            self._ftdi.enable_adaptive_clock(False)
            self._ftdi.enable_3phase_clock(True)
            try:
                self._ftdi.enable_drivezero_mode(self.MDC_BIT |
                                                 self.MDIO_O_BIT |
                                                 self.MDIO_I_BIT)
            except FtdiFeatureError:
                # when open collector feature is not available (FT2232, FT4232)
                # SDA line is temporary move to high-z to enable ACK/NACK
                # read back from slave
                self._fake_tristate = True
            self._wide_port = self._ftdi.has_wide_port
            if not self._wide_port:
                self._set_gpio_direction(8, 0, 0)


    def close(self, freeze: bool = False) -> None:
        """Close the FTDI interface.

           :param freeze: if set, FTDI port is not reset to its default
                          state on close.
        """
        with self._lock:
            if self._ftdi.is_connected:
                self._ftdi.close(freeze)

    def terminate(self) -> None:
        """Close the FTDI interface.

           :note: deprecated API, use close()
        """
        self.close()

    def get_port(self, address: int) -> MdioPort:
        """Obtain an MdioPort to drive an Mdio slave.

           :param address: the address on the MDIO bus
           :return: an MdioPort instance
        """
        if not self._ftdi.is_connected:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address not in self._slaves:
            self._slaves[address] = MdioPort(self, address)
        return self._slaves[address]


    @property
    def ftdi(self) -> Ftdi:
        """Return the Ftdi instance.

           :return: the Ftdi instance
        """
        return self._ftdi

    @property
    def configured(self) -> bool:
        """Test whether the device has been properly configured.

           :return: True if configured
        """
        #return self._ftdi.is_connected and bool(self._start)
        return self._ftdi.is_connected #and bool(self._start)

    @classmethod
    def validate_address(cls, address: Optional[int]) -> None:
        """Assert an MDIO slave address is in the supported range.
           None is a special bypass address.

           :param address: the address on the MDIO bus
           :raise MdioIOError: if the MDIO slave address is not supported
        """
        if address is None:
            return
        if address > 32:
            raise MdioIOError("No such Mdio slave: 0x%02x" % address)

    @property
    def frequency_max(self) -> float:
        """Provides the maximum MDIO clock frequency in Hz.

           :return: MDIO bus clock frequency
        """
        return self._ftdi.frequency_max

    @property
    def frequency(self) -> float:
        """Provides the current MDIO clock frequency in Hz.

           :return: the MDIO bus clock frequency
        """
        return self._frequency

    @property
    def direction(self) -> int:
        """Provide the FTDI pin direction

           A true bit represents an output pin, a false bit an input pin.

           :return: the bitfield of direction.
        """
        return self.MDIO_DIR | self._gpio_dir

    @property
    def gpio_pins(self) -> int:
        """Report the configured GPIOs as a bitfield.

           A true bit represents a GPIO, a false bit a reserved or not
           configured pin.

           :return: the bitfield of configured GPIO pins.
        """
        with self._lock:
            return self._gpio_mask

    @property
    def gpio_all_pins(self) -> int:
        """Report the addressable GPIOs as a bitfield.

           A true bit represents a pin which may be used as a GPIO, a false bit
           a reserved pin (for MDIO support)

           :return: the bitfield of configurable GPIO pins.
        """
        mask = (1 << self.width) - 1
        with self._lock:
            return mask & ~self._mdio_mask

    @property
    def width(self) -> int:
        """Report the FTDI count of addressable pins.

           :return: the count of IO pins (including MDIO ones).
        """
        return 16 if self._wide_port else 8

    def read(self, address: int, readlen: int = 1) -> bytes:
        """Read one or more bytes from a remote slave

           :param address: the address on the MDIO bus, or None to discard accessByClass22
           :param readlen: count of bytes to read out.
           :return: read bytes
           :raise MdioIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)

           Most MDIO devices require a register address to read out
           check out the exchange() method.
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            mdioaddress = None
        else:
            mdioaddress = address
        with self._lock:
            self._do_prolog(mdioaddress)
            data = self._do_read(readlen)
            self._do_epilog()
            return data


    def exchange(self, address: int,
                 out: Union[bytes, bytearray, Iterable[int]],
                 readlen: int = 0) -> bytes:
        """Send a byte sequence to a remote slave followed with
           a read request of one or more bytes.

           This command is useful to tell the slave what data
           should be read out.

           :param address: the address on the MDIO bus, or None to discard accessByClass22
           :param out: the byte buffer to send
           :param readlen: count of bytes to read out.
           :return: read bytes
           :raise MdioIOError: if device is not configured or input parameters
                              are invalid

           Address is a logical slave address (0x7f max)
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if readlen < 1:
            raise MdioIOError('Nothing to read')
        if readlen > (self.PAYLOAD_MAX_LENGTH/3-1):
            raise MdioIOError("Input payload is too large")
        if address is None:
            mdioaddress = None
        else:
            mdioaddress = (address << 1) & self.HIGH
        with self._lock:
            self._do_prolog(mdioaddress)
            self._do_write(out)
            self._do_prolog(mdioaddress | self.BIT0)
            if readlen:
                data = self._do_read(readlen)
            self._do_epilog()
            return data

    def poll(self, address: int, write: bool = False
             ) -> bool:
        """Poll a remote slave, expect ACK or NACK.

           :param address: the address on the MDIO bus, or None to discard accessByClass22
           :param write: poll in write mode (vs. read)
           :return: True if the slave acknowledged, False otherwise
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            mdioaddress = None
        else:
            mdioaddress = (address << 1) & self.HIGH
            if not write:
                mdioaddress |= self.BIT0
        with self._lock:
            try:
                self._do_prolog(mdioaddress)
                return True
            except MdioNackError:
                self.log.info('Not ready')
                return False
            finally:
                self._do_epilog()

    def poll_cond(self, address: int, fmt: str, mask: int, value: int,
                  count: int) -> Optional[bytes]:
        """Poll a remove slave, watching for condition to satisfy.
           On each poll cycle, a repeated accessByClass22 condition is emitted, without
           releasing the MDIO bus, and an ACK is returned to the slave.


           :param address: the address on the MDIO bus, or None to discard accessByClass22
           :param fmt: struct format for poll register
           :param mask: binary mask to apply on the condition register
                before testing for the value
           :param value: value to test the masked condition register
                against. Condition is satisfied when register & mask == value
           :param count: maximum poll count before raising a timeout
           :return: the polled register value, or None if poll failed
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            mdioaddress = None
        else:
            mdioaddress = (address << 1) & self.HIGH
            mdioaddress |= self.BIT0
        with self._lock:
            try:
                retry = 0
                while retry < count:
                    retry += 1
                    size = scalc(fmt)
                    self._do_prolog(mdioaddress)
                    data = self._do_read(size)
                    self.log.debug("Poll data: %s", hexlify(data).decode())
                    cond, = sunpack(fmt, data)
                    if (cond & mask) == value:
                        self.log.debug('Poll condition matched')
                        break
                    data = None
                    self.log.debug('Poll condition not fulfilled: %x/%x',
                                   cond & mask, value)
                if not data:
                    self.log.warning('Poll condition failed')
                return data
            except MdioNackError:
                self.log.info('Not ready')
                return None
            finally:
                self._do_epilog()

    def flush(self) -> None:
        """Flush the HW FIFOs.
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        with self._lock:
            self._ftdi.write_data(self._immediate)
            self._ftdi.purge_buffers()

    def read_gpio(self, with_output: bool = False) -> int:
        """Read GPIO port.

           :param with_output: set to unmask output pins
           :return: the GPIO port pins as a bitfield
        """
        with self._lock:
            data = self._read_raw(self._wide_port)
        value = data & self._gpio_mask
        if not with_output:
            value &= ~self._gpio_dir
        return value

    def write_gpio(self, value: int) -> None:
        """Write GPIO port.

           :param value: the GPIO port pins as a bitfield
        """
        with self._lock:
            if (value & self._gpio_dir) != value:
                raise MdioIOError('No such GPO pins: %04x/%04x' %
                                 (self._gpio_dir, value))
            # perform read-modify-write
            use_high = self._wide_port and (self.direction & 0xff00)
            data = self._read_raw(use_high)
            data &= ~self._gpio_mask
            data |= value
            self._write_raw(data, use_high)
            self._gpio_low = data & 0xFF & ~self._mdio_mask

    def set_gpio_direction(self, pins: int, direction: int) -> None:
        """Change the direction of the GPIO pins.

           :param pins: which GPIO pins should be reconfigured
           :param direction: direction bitfield (on for output)
        """
        with self._lock:
            self._set_gpio_direction(16 if self._wide_port else 8,
                                     pins, direction)

    def _set_gpio_direction(self, width: int, pins: int,
                            direction: int) -> None:
        if pins & self._mdio_mask:
            raise MdioIOError('Cannot access MDIO pins as GPIO')
        gpio_mask = (1 << width) - 1
        gpio_mask &= ~self._mdio_mask
        if (pins & gpio_mask) != pins:
            raise MdioIOError('No such GPIO pin(s)')
        self._gpio_dir &= ~pins
        self._gpio_dir |= (pins & direction)
        self._gpio_mask = gpio_mask & pins

    @property
    def _data_lo(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self.MDC_BIT | self._gpio_low,
                self.MDIO_DIR | (self._gpio_dir & 0xFF))

    @property
    def _clk_lo_data_hi(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self.MDIO_O_BIT | self._gpio_low,
                self.MDIO_DIR | (self._gpio_dir & 0xFF))

    @property
    def _clk_lo_data_input(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self.LOW | self._gpio_low,
                self.MDC_BIT | (self._gpio_dir & 0xFF))

    @property
    def _clk_lo_data_lo(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self._gpio_low,
                self.MDIO_DIR | (self._gpio_dir & 0xFF))

    @property
    def _clk_input_data_input(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self._gpio_low,
                (self._gpio_dir & 0xFF))

    @property
    def _idle(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW,
                self.MDIO_DIR | self._gpio_low,
                self.MDIO_DIR | (self._gpio_dir & 0xFF))

    @property
    def _start(self) -> Tuple[int]:
        return self._data_lo * self._ck_hd_sta + \
               self._clk_lo_data_lo * self._ck_hd_sta

    @property
    def _stop(self) -> Tuple[int]:
        return self._clk_lo_data_hi * self._ck_hd_sta + \
               self._clk_lo_data_lo * self._ck_hd_sta + \
               self._data_lo * self._ck_su_sto + \
               self._idle * self._ck_idle

    def _compute_delay_cycles(self, value: Union[int, float]) -> int:
        # approx ceiling without relying on math module
        # the bit delay is far from being precisely known anyway
        bit_delay = self._ftdi.mpsse_bit_delay
        return max(1, int((value + bit_delay) / bit_delay))

    def _read_raw(self, read_high: bool) -> int:
        if read_high:
            cmd = bytes([Ftdi.GET_BITS_LOW,
                         Ftdi.GET_BITS_HIGH,
                         Ftdi.SEND_IMMEDIATE])
            fmt = '<H'
        else:
            cmd = bytes([Ftdi.GET_BITS_LOW,
                         Ftdi.SEND_IMMEDIATE])
            fmt = 'B'
        self._ftdi.write_data(cmd)
        size = scalc(fmt)
        data = self._ftdi.read_data_bytes(size, 4)
        if len(data) != size:
            raise MdioIOError('Cannot read GPIO')
        value, = sunpack(fmt, data)
        return value

    def _write_raw(self, data: int, write_high: bool):
        direction = self.direction
        low_data = data & 0xFF
        low_dir = direction & 0xFF
        if write_high:
            high_data = (data >> 8) & 0xFF
            high_dir = (direction >> 8) & 0xFF
            cmd = bytes([Ftdi.SET_BITS_LOW, low_data, low_dir,
                         Ftdi.SET_BITS_HIGH, high_data, high_dir])
        else:
            cmd = bytes([Ftdi.SET_BITS_LOW, low_data, low_dir])
        self._ftdi.write_data(cmd)

    def _do_prolog(self, mdioaddress: int) -> None:
        if mdioaddress is None:
            return
        self.log.debug('   prolog 0x%x', mdioaddress >> 1)
        cmd = bytearray(self._idle * self._ck_delay)
        cmd.extend(self._start)
        cmd.extend(self._write_byte)
        cmd.append(mdioaddress)
        try:
            self._send_check_ack(cmd)
        except MdioNackError:
            self.log.warning('NACK @ 0x%02x', (mdioaddress >> 1))
            raise

    def _do_epilog(self) -> None:
        self.log.debug('   epilog')
        cmd = bytearray(self._stop)
        if self._fake_tristate:
            # SCL high-Z, SDA high-Z
            cmd.extend(self._clk_input_data_input)
        self._ftdi.write_data(cmd)
        # be sure to purge the MPSSE reply
        self._ftdi.read_data_bytes(1, 1)

    def _send_check_ack(self, cmd: bytearray):
        # note: cmd is modified
        if self._fake_tristate:
            # SCL low, SDA high-Z (input)
            cmd.extend(self._clk_lo_data_input)
            # read SDA (ack from slave)
            cmd.extend(self._read_bit)
        else:
            # SCL low, SDA high-Z
            cmd.extend(self._clk_lo_data_hi)
            # read SDA (ack from slave)
            cmd.extend(self._read_bit)
        cmd.extend(self._immediate)
        self._ftdi.write_data(cmd)
        ack = self._ftdi.read_data_bytes(1, 4)
        if not ack:
            raise MdioIOError('No answer from FTDI')
        if ack[0] & self.BIT0:
            raise MdioNackError('NACK from slave')

    def _do_read(self, readlen: int) -> bytes:
        self.log.debug('- read %d byte(s)', readlen)
        if not readlen:
            # force a real read request on device, but discard any result
            cmd = bytearray()
            cmd.extend(self._immediate)
            self._ftdi.write_data(cmd)
            self._ftdi.read_data_bytes(0, 4)
            return bytearray()
        if self._fake_tristate:
            read_byte = (self._clk_lo_data_input +
                         self._read_byte +
                         self._clk_lo_data_hi)
            read_not_last = (read_byte + self._ack +
                             self._clk_lo_data_lo * self._ck_delay)
            read_last = (read_byte + self._nack +
                         self._clk_lo_data_hi * self._ck_delay)
        else:
            read_not_last = (self._read_byte + self._ack +
                             self._clk_lo_data_hi * self._ck_delay)
            read_last = (self._read_byte + self._nack +
                         self._clk_lo_data_hi * self._ck_delay)
        # maximum RX size to fit in FTDI FIFO, minus 2 status bytes
        chunk_size = self._rx_size-2
        cmd_size = len(read_last)
        # limit RX chunk size to the count of MDIO packable commands in the FTDI
        # TX FIFO (minus one byte for the last 'send immediate' command)
        tx_count = (self._tx_size-1) // cmd_size
        chunk_size = min(tx_count, chunk_size)
        chunks = []
        cmd = None
        rem = readlen
        if self._read_optim and rem > chunk_size:
            chunk_size //= 2
            self.log.debug('Use optimized transfer, %d byte at a time',
                           chunk_size)
            cmd_chunk = bytearray()
            cmd_chunk.extend(read_not_last * chunk_size)
            cmd_chunk.extend(self._immediate)
            def write_command_gen(length: int):
                if length <= 0:
                    # no more data
                    return b''
                if length <= chunk_size:
                    cmd = bytearray()
                    cmd.extend(read_not_last * (length-1))
                    cmd.extend(read_last)
                    cmd.extend(self._immediate)
                    return cmd
                return cmd_chunk

            while rem:
                buf = self._ftdi.read_data_bytes(rem, 4, write_command_gen)
                self.log.debug('- read %d bytes, rem: %d', len(buf), rem)
                chunks.append(buf)
                rem -= len(buf)
        else:
            while rem:
                if rem > chunk_size:
                    if not cmd:
                        # build the command sequence only once, as it may be
                        # repeated till the end of the transfer
                        cmd = bytearray()
                        cmd.extend(read_not_last * chunk_size)
                        size = chunk_size
                else:
                    cmd = bytearray()
                    cmd.extend(read_not_last * (rem-1))
                    cmd.extend(read_last)
                    cmd.extend(self._immediate)
                    size = rem
                self._ftdi.write_data(cmd)
                buf = self._ftdi.read_data_bytes(size, 4)
                self.log.debug('- read %d byte(s): %s',
                               len(buf), hexlify(buf).decode())
                chunks.append(buf)
                rem -= size
        return bytearray(b''.join(chunks))

########################################################################################
##################### write by masw : masw@masw.tech ###################################
# 1. not support GPIO !
# 
    @property
    def _clk_lo_data_hi(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW              ,  # ftdi command
                self.MDIO_O_BIT                ,  # output value
                self.MDIO_DIR                  )  # 1: output; 0: input
    @property
    def _clk_hi_data_hi(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW              ,  # ftdi command
                self.MDIO_O_BIT | self.MDC_BIT ,  # output value
                self.MDIO_DIR                  )  # 1: output; 0: input
    @property
    def _clk_lo_data_lo(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW              ,  # ftdi command
                0x0                            ,  # output value
                self.MDIO_DIR                  )  # 1: output; 0: input
    @property
    def _clk_hi_data_lo(self) -> Tuple[int]:
        return (Ftdi.SET_BITS_LOW              ,  # ftdi command
                self.MDC_BIT                   ,  # output value
                self.MDIO_DIR                  )  # 1: output; 0: input

    
    @property
    def _preamble(self) -> Tuple[int]:
        preamble = ()
        for i in range(32):
            preamble += self._clk_lo_data_hi 
            preamble += self._clk_hi_data_hi 
        return preamble

    def _do_write(self, out: Union[bytes, bytearray, Iterable[int]]):
        if not isinstance(out, bytearray):
            out = bytearray(out)
        if not out:
            return
        self.log.debug('- write %d byte(s): %s',
                       len(out), hexlify(out).decode())
        for byte in out:
            cmd = bytearray(self._preamble)
            self._ftdi.write_data(cmd)

    def write(self, address: int, out: Union[bytes, bytearray, Iterable[int]]) -> None:
        """Write one or more bytes to a remote slave
        """
        if not self.configured:
            raise MdioIOError("FTDI controller not initialized")
        self.validate_address(address)
        if address is None:
            mdioaddress = None
        else:
            mdioaddress = address
        with self._lock:
            self._do_write(out)
            return

# 测试主函数
if __name__ == '__main__':
    ctrl = MdioController()
    ctrl.configure('ftdi://ftdi:232h/1')
    mdio = ctrl.get_port(0x0)
    # send 2 bytes
    mdio.write_to(0x001, [0x12, 0x34])
