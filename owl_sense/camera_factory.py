import importlib, yaml
from typing import Any, Dict
from .camera_base import CameraBase

def _load_yaml(path: str) -> Dict[str, Any]:
    if not path:
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}

def load_camera(impl: str, driver_params_file: str = "", **kwargs: Any) -> CameraBase:
    """
    impl e.g.: 'owl_sense.camera_ar082:CameraAR082x'
    kwargs are generic (e.g., camera_name) and not camera-specific knobs.
    """
    mod_name, _, cls_name = impl.partition(':')
    if not (mod_name and cls_name):
        raise ValueError("camera_impl must be 'module.path:ClassName'")
    module = importlib.import_module(mod_name)
    cls = getattr(module, cls_name)

    cfg = _load_yaml(driver_params_file)
    init_args = {**kwargs, **cfg.get("init", {})}

    cam: CameraBase = cls(**init_args)
    if not isinstance(cam, CameraBase):
        raise TypeError(f"{cls_name} does not implement CameraBase")

    # Stash config sections for later phases
    cam._pre_cfg = cfg.get("pre_stream", {})
    cam._run_cfg = cfg.get("runtime", {})
    return cam

