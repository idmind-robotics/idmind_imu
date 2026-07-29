"""
A fake BrickDaemon TCP server for testing the ``tinkerforge`` bindings offline.

This is a test *helper*, not a test module: it implements just enough of the BrickDaemon wire
protocol (see ``tinkerforge.ip_connection``) to convince the real, unmodified bindings that a
genuine IMU Brick 2.0 is attached, without any hardware or ``brickd`` process. It is imported
by ``test_fake_brickd.py`` (which proves the fake is convincing) and by
``test_brick_v2_driver.py`` (which exercises the real driver against it).

The wire protocol is an 8-byte little-endian header (uid, length, function_id,
seq/response-expected, error/future) followed by an optional payload, framed purely by the
length byte. There is no handshake and no magic bytes.
"""

import socket
import struct
import threading

from tinkerforge.ip_connection import base58decode, pack_payload, unpack_payload

#: Mirrors ``IPConnection.FUNCTION_ENUMERATE``.
FUNCTION_ENUMERATE = 254
#: Mirrors ``IPConnection.CALLBACK_ENUMERATE``.
CALLBACK_ENUMERATE = 253
#: Mirrors ``IPConnection.FUNCTION_DISCONNECT_PROBE`` (sent ~every 5s on an idle connection).
FUNCTION_DISCONNECT_PROBE = 128
#: Mirrors ``Device.FUNCTION_GET_IDENTITY``, common to every TinkerForge device.
FUNCTION_GET_IDENTITY = 255
#: Mirrors ``BrickIMUV2.DEVICE_IDENTIFIER``.
DEVICE_IDENTIFIER_IMU_V2 = 18
#: Mirrors ``BrickIMUV2.CALLBACK_ALL_DATA``.
CALLBACK_ALL_DATA = 40

# IMU Brick 2.0 function ids the driver actually exercises, from brick_imu_v2.py.
FUNCTION_LEDS_ON = 10
FUNCTION_LEDS_OFF = 11
FUNCTION_ARE_LEDS_ON = 12
FUNCTION_SET_ALL_DATA_PERIOD = 30
FUNCTION_GET_ALL_DATA_PERIOD = 31
FUNCTION_SET_SENSOR_FUSION_MODE = 43
FUNCTION_GET_SENSOR_FUSION_MODE = 44
FUNCTION_ENABLE_STATUS_LED = 238
FUNCTION_DISABLE_STATUS_LED = 239
FUNCTION_IS_STATUS_LED_ENABLED = 240

#: Reply format for both ``CALLBACK_ENUMERATE`` and ``GET_IDENTITY``, minus the trailing
#: ``enumeration_type`` byte that only enumerate replies carry.
_IDENTITY_FORMAT = '8s 8s c 3B 3B H'
#: Payload format for ``BrickIMUV2.CALLBACK_ALL_DATA`` (54 bytes total, header included).
_ALL_DATA_FORMAT = '3h 3h 3h 3h 4h 3h 3h b B'


def _recv_exact(sock, size):
    """Read exactly ``size`` bytes from ``sock``, or return ``b''`` if the peer closed."""
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            return b''
        chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(chunks)


def _handle_set_all_data_period(state, payload):
    """Apply ``SET_ALL_DATA_PERIOD``: store the requested period, no reply payload."""
    # unpack_payload collapses a single-field format to a bare scalar, not a 1-tuple.
    state['all_data_period'] = unpack_payload(payload, 'I')
    return ()


def _handle_get_all_data_period(state, _payload):
    """Answer ``GET_ALL_DATA_PERIOD`` with the currently stored period."""
    return (state['all_data_period'],)


def _handle_set_sensor_fusion_mode(state, payload):
    """Apply ``SET_SENSOR_FUSION_MODE``: store the requested mode, no reply payload."""
    state['sensor_fusion_mode'] = unpack_payload(payload, 'B')
    return ()


def _handle_get_sensor_fusion_mode(state, _payload):
    """Answer ``GET_SENSOR_FUSION_MODE`` with the currently stored mode."""
    return (state['sensor_fusion_mode'],)


def _handle_leds_on(state, _payload):
    """Apply ``LEDS_ON``: mark the orientation/direction LEDs on, no reply payload."""
    state['leds_on'] = True
    return ()


def _handle_leds_off(state, _payload):
    """Apply ``LEDS_OFF``: mark the orientation/direction LEDs off, no reply payload."""
    state['leds_on'] = False
    return ()


def _handle_are_leds_on(state, _payload):
    """Answer ``ARE_LEDS_ON`` with the currently stored LED state."""
    return (state['leds_on'],)


def _handle_enable_status_led(state, _payload):
    """Apply ``ENABLE_STATUS_LED``, no reply payload."""
    state['status_led_enabled'] = True
    return ()


def _handle_disable_status_led(state, _payload):
    """Apply ``DISABLE_STATUS_LED``, no reply payload."""
    state['status_led_enabled'] = False
    return ()


def _handle_is_status_led_enabled(state, _payload):
    """Answer ``IS_STATUS_LED_ENABLED`` with the currently stored status LED state."""
    return (state['status_led_enabled'],)


#: Table-driven dispatch: function_id -> (reply payload format, handler(state, payload)).
#: The handler both applies the request to the internal state dict and returns the tuple of
#: values to pack into the reply, so a setter really does change what the matching getter
#: returns later.
_HANDLERS = {
    FUNCTION_SET_ALL_DATA_PERIOD: ('', _handle_set_all_data_period),
    FUNCTION_GET_ALL_DATA_PERIOD: ('I', _handle_get_all_data_period),
    FUNCTION_SET_SENSOR_FUSION_MODE: ('', _handle_set_sensor_fusion_mode),
    FUNCTION_GET_SENSOR_FUSION_MODE: ('B', _handle_get_sensor_fusion_mode),
    FUNCTION_LEDS_ON: ('', _handle_leds_on),
    FUNCTION_LEDS_OFF: ('', _handle_leds_off),
    FUNCTION_ARE_LEDS_ON: ('!', _handle_are_leds_on),
    FUNCTION_ENABLE_STATUS_LED: ('', _handle_enable_status_led),
    FUNCTION_DISABLE_STATUS_LED: ('', _handle_disable_status_led),
    FUNCTION_IS_STATUS_LED_ENABLED: ('!', _handle_is_status_led_enabled),
}


class FakeBrickd:
    """
    A minimal fake BrickDaemon that speaks just enough protocol to fool the real bindings.

    Binds to an ephemeral port on ``127.0.0.1`` (read back via ``.port``) so tests never
    collide on a fixed port. Usable as a context manager or via explicit ``start()``/``stop()``.
    Handles ``ENUMERATE``, ``GET_IDENTITY`` (the classic ``check_validity()`` trap), the IMU
    Brick 2.0 getters/setters the driver uses, and ignores ``DISCONNECT_PROBE`` rather than
    closing the connection. ``push_all_data()`` lets a test inject an unsolicited
    ``CALLBACK_ALL_DATA`` frame at will.
    """

    def __init__(self, uid="6Dpwed", device_identifier=DEVICE_IDENTIFIER_IMU_V2, host="127.0.0.1"):
        """Create the fake; does not bind or listen until ``start()`` is called."""
        self.uid = uid
        self.uid_num = base58decode(uid)
        self.device_identifier = device_identifier
        self._host = host
        self.port = None

        self._state = {
            'all_data_period': 0,
            'sensor_fusion_mode': 2,
            'leds_on': False,
            'status_led_enabled': True,
        }

        self.requests = []
        self._requests_lock = threading.Lock()

        self._server_sock = None
        self._client_sock = None
        self._client_lock = threading.Lock()
        self._accept_thread = None
        self._client_thread = None
        self._stop_event = threading.Event()

    # -- lifecycle -------------------------------------------------------------------------

    def start(self):
        """Bind an ephemeral port and start accepting connections on a daemon thread."""
        self._stop_event.clear()
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self._host, 0))
        self._server_sock.listen(1)
        self.port = self._server_sock.getsockname()[1]

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()
        return self

    def stop(self):
        """Close every socket and join every thread. Idempotent: safe to call twice."""
        self._stop_event.set()

        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None

        with self._client_lock:
            client_sock = self._client_sock
            self._client_sock = None
        if client_sock is not None:
            try:
                client_sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                client_sock.close()
            except OSError:
                pass

        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)
            self._accept_thread = None
        if self._client_thread is not None:
            self._client_thread.join(timeout=2.0)
            self._client_thread = None

    def __enter__(self):
        """Start the fake and return it, for ``with FakeBrickd() as fake:``."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Stop the fake on context-manager exit, regardless of how the block ended."""
        self.stop()

    # -- accept / receive loops --------------------------------------------------------------

    def _accept_loop(self):
        """Accept a single client connection at a time and service it until it disconnects."""
        while not self._stop_event.is_set():
            try:
                conn, _addr = self._server_sock.accept()
            except OSError:
                return

            with self._client_lock:
                self._client_sock = conn

            self._client_thread = threading.Thread(
                target=self._client_loop, args=(conn,), daemon=True
            )
            self._client_thread.start()

    def _client_loop(self, conn):
        """Read framed packets from one client and dispatch each until it disconnects."""
        try:
            while not self._stop_event.is_set():
                header = _recv_exact(conn, 8)
                if not header:
                    return
                length = header[4]
                payload = _recv_exact(conn, length - 8) if length > 8 else b''
                if length > 8 and not payload:
                    return
                self._dispatch(conn, header, payload)
        except OSError:
            return
        finally:
            with self._client_lock:
                if self._client_sock is conn:
                    self._client_sock = None

    # -- request dispatch --------------------------------------------------------------------

    def _dispatch(self, conn, header, payload):
        """Handle one request packet: enumerate, disconnect-probe, identity, or a getter/setter."""
        uid_num, _length, function_id, seq_options, _error = struct.unpack('<IBBBB', header)
        sequence_number = (seq_options >> 4) & 0x0F
        response_expected = bool(seq_options & 0x08)

        with self._requests_lock:
            self.requests.append(function_id)

        if function_id == FUNCTION_ENUMERATE:
            self._reply_enumerate(conn)
            return

        if function_id == FUNCTION_DISCONNECT_PROBE:
            # Idle-connection keepalive: the client expects no reply whatsoever.
            return

        if function_id == FUNCTION_GET_IDENTITY:
            self._reply_identity(conn, sequence_number)
            return

        handler_entry = _HANDLERS.get(function_id)
        if handler_entry is None:
            return

        reply_format, handler = handler_entry
        values = handler(self._state, payload)
        if response_expected:
            body = pack_payload(values, reply_format) if reply_format else b''
            self._send_packet(conn, self.uid_num, function_id, sequence_number, body)

    # -- reply builders ---------------------------------------------------------------------

    def _identity_payload(self):
        """Build the shared ``(uid, connected_uid, position, hw, fw, device_identifier)`` body."""
        return pack_payload(
            (self.uid, "0", "0", (2, 0, 0), (2, 0, 0), self.device_identifier),
            _IDENTITY_FORMAT,
        )

    def _reply_enumerate(self, conn):
        """Send one unsolicited ``CALLBACK_ENUMERATE`` packet (seq 0), 34 bytes total."""
        body = self._identity_payload() + struct.pack('<B', 0)  # enumeration_type = AVAILABLE
        self._send_packet(conn, 0, CALLBACK_ENUMERATE, 0, body)

    def _reply_identity(self, conn, sequence_number):
        """Answer ``GET_IDENTITY`` so ``Device.check_validity()`` accepts this device."""
        self._send_packet(
            conn, self.uid_num, FUNCTION_GET_IDENTITY, sequence_number, self._identity_payload()
        )

    def _send_packet(self, conn, uid_num, function_id, sequence_number, body):
        """Assemble and write one 8-byte-header packet directly on ``conn``."""
        length = 8 + len(body)
        seq_options = (sequence_number << 4) | 0  # response bit is meaningless in a reply
        header = struct.pack('<IBBBB', uid_num, length, function_id, seq_options, 0)
        conn.sendall(header + body)

    # -- test-facing API ----------------------------------------------------------------------

    def push_all_data(
        self,
        acceleration=(0, 0, 0),
        magnetic_field=(0, 0, 0),
        angular_velocity=(0, 0, 0),
        euler_angle=(0, 0, 0),
        quaternion=(16383, 0, 0, 0),
        linear_acceleration=(0, 0, 0),
        gravity_vector=(0, 0, 0),
        temperature=0,
        calibration_status=0,
    ):
        """
        Push one unsolicited ``CALLBACK_ALL_DATA`` frame to the connected client, if any.

        Every field defaults to zero (quaternion defaults to the identity rotation) so a test
        can override just the field it cares about. A no-op, not an error, if no client is
        currently connected.
        """
        body = pack_payload(
            (
                tuple(acceleration),
                tuple(magnetic_field),
                tuple(angular_velocity),
                tuple(euler_angle),
                tuple(quaternion),
                tuple(linear_acceleration),
                tuple(gravity_vector),
                temperature,
                calibration_status,
            ),
            _ALL_DATA_FORMAT,
        )

        with self._client_lock:
            conn = self._client_sock
        if conn is None:
            return
        try:
            self._send_packet(conn, self.uid_num, CALLBACK_ALL_DATA, 0, body)
        except OSError:
            pass
