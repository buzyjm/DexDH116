"""Deployed package entry for RS485-only examples."""

from .controller_rs485 import RS485Controller

LHandProController = RS485Controller

__all__ = ["LHandProController", "RS485Controller"]
