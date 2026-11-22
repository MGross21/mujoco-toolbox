"""Results of a simulation run: captured data, rendered frames, and their export."""

from __future__ import annotations

import math
import sys
import tempfile
import warnings
import webbrowser
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING, Self

import numpy as np

from .errors import SimulationError, SimulationWarning

if TYPE_CHECKING:
    import os

    import mujoco
    from IPython.display import Video

# MjData fields indexed by body id along their first axis.
_BODY_FIELDS = frozenset(
    {
        "xpos",
        "xquat",
        "xmat",
        "xipos",
        "ximat",
        "subtree_com",
        "cinert",
        "crb",
        "cvel",
        "cacc",
        "cfrc_int",
        "cfrc_ext",
        "xfrc_applied",
        "subtree_linvel",
        "subtree_angmom",
    },
)


@dataclass(frozen=True, slots=True, repr=False)
class Recording:
    """What one :meth:`~mujoco_toolbox.sim.Simulation.run` captured.

    Example:
        >>> rec = sim.run(duration=5, render=True)
        >>> rec.data["qpos"]  # (captures, nq)
        >>> rec.clip(1, 2).save("middle.mp4")

    """

    model: mujoco.MjModel
    """The model that was simulated."""
    time: np.ndarray
    """Simulated time of each capture, shape ``(captures,)``."""
    data: dict[str, np.ndarray | list[np.ndarray]]
    """Captured ``MjData`` fields, each ``(captures, *field_shape)``; lists for fields that vary in size."""
    frames: np.ndarray | None = None
    """Rendered frames ``(n, height, width, 3)``, or ``None`` without ``render=True``."""
    frame_times: np.ndarray | None = None
    """Simulated time of each frame, shape ``(n,)``."""
    fps: float = 30
    """Playback rate of :attr:`frames`, in frames per simulated second."""

    def __repr__(self) -> str:
        frames = 0 if self.frames is None else len(self.frames)
        return f"Recording({len(self.time)} captures, {len(self.data)} fields, {frames} frames)"

    def clip(self, start: float = 0, end: float = math.inf) -> Self:
        """The part of this recording between ``start`` and ``end`` seconds, inclusive."""
        keep = (self.time >= start) & (self.time <= end)
        data = {
            k: v[keep] if isinstance(v, np.ndarray) else [x for x, kept in zip(v, keep, strict=True) if kept]
            for k, v in self.data.items()
        }
        if self.frames is None or self.frame_times is None:
            return replace(self, time=self.time[keep], data=data)
        shown = (self.frame_times >= start) & (self.frame_times <= end)
        return replace(
            self, time=self.time[keep], data=data, frames=self.frames[shown], frame_times=self.frame_times[shown]
        )

    def body(self, name: str, field: str | None = None) -> np.ndarray | dict[str, np.ndarray]:
        """Captured per-body fields (``xpos``, ``xquat``, ``cvel``, ...) of one body.

        Args:
            name: Body name.
            field: Field to return. Defaults to every captured per-body field.

        Returns:
            ``(captures, *field_shape)`` for ``field``, or a dict of them keyed by field.

        Raises:
            KeyError: If the body is not in the model.
            ValueError: If ``field`` is not a captured per-body field.

        """
        body_id = self.model.body(name).id
        fields = {k: v[:, body_id] for k, v in self.data.items() if k in _BODY_FIELDS and isinstance(v, np.ndarray)}
        if field is None:
            return fields
        if field not in fields:
            msg = f"'{field}' is not a captured per-body field. Captured: {', '.join(sorted(fields)) or 'none'}."
            raise ValueError(msg)
        return fields[field]

    def _rendered(self) -> tuple[np.ndarray, np.ndarray]:
        if self.frames is None or self.frame_times is None or not len(self.frames):
            msg = "No frames recorded. Run with render=True."
            raise SimulationError(msg)
        return self.frames, self.frame_times

    def frame_at(self, t: float) -> np.ndarray:
        """The last frame rendered at or before ``t`` seconds."""
        frames, times = self._rendered()
        return frames[max(0, int(np.searchsorted(times, t, side="right")) - 1)]

    def save(self, path: str | os.PathLike[str] = "output.gif") -> str:
        """Write the frames to a video or GIF; the extension picks the format.

        Returns:
            Absolute path of the written file.

        Raises:
            SimulationError: If nothing was rendered or writing fails.

        """
        frames, _ = self._rendered()
        import imageio.v3 as iio  # noqa: PLC0415

        fps = self.fps if len(frames) > 1 else 1
        timing = {"duration": 1000 / fps} if Path(path).suffix.lower() == ".gif" else {"fps": fps}
        try:
            iio.imwrite(Path(path), frames, **timing)
        except Exception as e:
            msg = f"Could not write {path}."
            raise SimulationError(msg) from e
        return str(Path(path).resolve())

    def show(self) -> Video | None:
        """Play the frames inline in Jupyter, or in the system's video player elsewhere.

        Returns:
            An inline video in Jupyter, otherwise ``None``.

        Raises:
            SimulationError: If nothing was rendered.

        """
        frames, _ = self._rendered()
        if "ipykernel" in sys.modules:
            import imageio.v3 as iio  # noqa: PLC0415
            from IPython.display import Video  # noqa: PLC0415

            data = iio.imwrite("<bytes>", frames, extension=".mp4", fps=self.fps)
            return Video(data=data, embed=True, mimetype="video/mp4", html_attributes="controls autoplay loop")
        path = self.save(Path(tempfile.mkdtemp(prefix="mujoco-toolbox-")) / "recording.mp4")
        webbrowser.open(Path(path).as_uri())
        return None

    def save_data(self, path: str | os.PathLike[str]) -> str:
        """Write :attr:`time` and :attr:`data` to a ``.npz`` file, read back with ``np.load``.

        Fields that vary in size per capture are skipped with a warning.

        Returns:
            Absolute path of the written file.

        """
        arrays = {k: v for k, v in self.data.items() if isinstance(v, np.ndarray)}
        if skipped := sorted(self.data.keys() - arrays.keys()):
            warnings.warn(f"Skipped variable-size fields: {', '.join(skipped)}.", SimulationWarning, stacklevel=2)
        np.savez(path, allow_pickle=False, time=self.time, **arrays)
        return str(Path(path).with_suffix(".npz").resolve())
