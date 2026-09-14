"""Deployed package entry for EtherCAT-only examples."""

from .controller_ecat import EtherCATController

LHandProController = EtherCATController

__all__ = ["LHandProController", "EtherCATController"]
