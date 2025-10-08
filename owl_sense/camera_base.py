from abc import ABC, abstractmethod
from typing import Optional, Tuple, Literal, Dict, Any, TypedDict, List, Union

class Frame(TypedDict, total=False):
    kind: Literal["compressed", "raw"]      # REQUIRED
    data: bytes                              # REQUIRED for both
    encoding: str                            # REQUIRED for raw (e.g., "rgb8", "bayer_rg8")
    width: int                               # REQUIRED for raw
    height: int                              # REQUIRED for raw

FrameLike = Union[Frame, List[Frame]]

class CameraBase(ABC):
    @abstractmethod
    def open(self) -> None: ...
    @abstractmethod
    def configure_pre_stream(self, cfg: Dict[str, Any]) -> None: ...
    @abstractmethod
    def start_streaming(self) -> None: ...
    @abstractmethod
    def configure_runtime(self, cfg: Dict[str, Any]) -> None: ...
    @abstractmethod
    def stop_streaming(self) -> None: ...
    @abstractmethod
    def close(self) -> None: ...
    @abstractmethod
    def acquire(self, timeout_ms: int) -> Tuple[str, Optional[FrameLike]]: ...
    @abstractmethod
    def capabilities(self) -> Dict[str, set]: ...

