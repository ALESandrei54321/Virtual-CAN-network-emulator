# bus_broker/protocols/registry.py

from .base_protocol import BaseProtocol
from .can_protocol import CANProtocol
from .can_fd_protocol import CANFDProtocol

# ── Protocol registry ─────────────────────────────────────────────────────────
# To add a new protocol:
#   1. Create the protocol class in its own file
#   2. Import it here
#   3. Add one line to this dict
# Nothing else needs to change.

PROTOCOL_REGISTRY: dict[str, type[BaseProtocol]] = {
    "CAN":    CANProtocol,
    "CAN_FD": CANFDProtocol,
}


def get_protocol(name: str, **kwargs) -> BaseProtocol:
    """
    Instantiate a protocol by name.
    kwargs are passed to the protocol constructor.

    Example:
        proto = get_protocol("CAN", bit_rate=250_000)
        proto = get_protocol("CAN_FD", nominal_bit_rate=500_000,
                                        data_bit_rate=2_000_000)
    """
    if name not in PROTOCOL_REGISTRY:
        available = ", ".join(PROTOCOL_REGISTRY.keys())
        raise ValueError(
            f"Unknown protocol '{name}'. "
            f"Available: {available}"
        )
    return PROTOCOL_REGISTRY[name](**kwargs)


def list_protocols() -> list[str]:
    """Return names of all registered protocols."""
    return list(PROTOCOL_REGISTRY.keys())
