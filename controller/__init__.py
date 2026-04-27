try:
    from .nmpc_controller import NMPCConfig, NonlinearMPCController
except Exception:  # pragma: no cover
    pass
try:
    from .mpc_controller import MPCConfig, MPCController
except Exception:  # pragma: no cover
    pass
