"""Deployed package entry for CANFD-only examples."""

from .controller_canfd import CANFDController

LHandProController = CANFDController

__all__ = ["LHandProController", "CANFDController"]
