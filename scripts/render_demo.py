"""Render the demo video: camera, map and live error for a simulated GNSS blackout.

Every number on screen comes from the data. The seed is the 8-seed median case of the run without
vision, so the video shows neither the best nor the worst outcome. Use `--stills` to check the
layout at chosen video times without rendering the whole video.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from PIL import Image  # noqa: E402

from gnss_denied.core.types import CameraFrame  # noqa: E402
from gnss_denied.datasets.kitti_raw import KittiRawDrive  # noqa: E402
from gnss_denied.frontend.visual_odom import FrameOdometry, VoNoise, load_odometry  # noqa: E402
from gnss_denied.pipeline import load_imu_profile, run_fusion  # noqa: E402
from gnss_denied.simulator.gnss_degrader import (  # noqa: E402
    DegradationMode,
    GnssDegrader,
    load_degrader_config,
)
from gnss_denied.simulator.imu_degrader import corrupt_imu  # noqa: E402
from gnss_denied.viz.demo_timeline import (  # noqa: E402
    DemoTimeline,
    Segment,
    interpolate_path,
    median_case_seed,
)

WIDTH_PX, HEIGHT_PX, DPI = 1920, 1080, 100
FPS = 30
PROFILE = "mems"
SEEDS = 8
DRIVE_START_S = 10.0
ATTRIBUTION = "Data: KITTI Raw (Geiger et al., IJRR 2013), CC BY-NC-SA 3.0"

# Dark steps of the validated categorical palette, in its fixed order.
SURFACE = "#1a1a19"
TEXT_PRIMARY = "#f0efec"
TEXT_SECONDARY = "#c3c2b7"
GRID = "#383835"
BLACKOUT_BAND = "#2a2a28"
TRUTH = "#c3c2b7"
MODES = (
    ("GNSS+IMU (Blackout)", None, "GNSS + IMU", "#3987e5"),
    ("GNSS+IMU+Classic_ORB", "orb", "+ ORB", "#d95926"),
    ("GNSS+IMU+Deep_LightGlue", "aliked", "+ ALIKED + LightGlue", "#199e70"),
)
MATCH_COLOR = MODES[2][3]


@dataclass(frozen=True)
class Scene:
    """Everything the renderer needs, already computed."""

    timeline: DemoTimeline
    blackout: tuple[float, float]
    seed: int
    times_s: np.ndarray
    paths_m: dict[str, np.ndarray]
    truth_at_epochs_m: np.ndarray
    truth_path_m: np.ndarray
    frames: list[CameraFrame]
    match_offsets: np.ndarray
    match_points: np.ndarray
    inliers_by_frame: np.ndarray


def _frames_in_span(drive: KittiRawDrive, start_s: float, end_s: float) -> list[CameraFrame]:
    return [frame for frame in drive.frames(0) if start_s - 0.2 <= frame.timestamp <= end_s + 0.2]


def _load_matches(cache: Path, frames: list[CameraFrame]) -> tuple[np.ndarray, np.ndarray]:
    """LightGlue matches between consecutive frames, geometrically filtered for display.

    The odometry cache keeps only the resulting motion, so the matches are computed once here. The
    first entry for each frame has no predecessor and stays empty.
    """
    stamps = np.array([frame.timestamp for frame in frames])
    if cache.exists():
        with np.load(cache, allow_pickle=False) as data:
            if np.array_equal(data["timestamps"], stamps):
                return data["offsets"], data["points"]

    from gnss_denied.frontend.deep_features import AlikedLightGlueBackend

    backend = AlikedLightGlueBackend()
    offsets = [0]
    chunks = []
    previous = None
    for index, frame in enumerate(frames):
        features = backend.extract(cv2.imread(str(frame.image_path), cv2.IMREAD_GRAYSCALE))
        count = 0
        if previous is not None:
            matches = backend.match(previous, features)
            start, end = previous.keypoints[matches[:, 0]], features.keypoints[matches[:, 1]]
            if len(matches) >= 8:
                _, mask = cv2.findFundamentalMat(start, end, cv2.USAC_MAGSAC, 1.0, 0.999)
                keep = mask.ravel().astype(bool) if mask is not None else np.zeros(0, dtype=bool)
                if keep.size:
                    chunks.append(np.hstack([start[keep], end[keep]]))
                    count = int(keep.sum())
        offsets.append(offsets[-1] + count)
        previous = features
        if (index + 1) % 100 == 0:
            print(f"  matches: {index + 1}/{len(frames)} frames", flush=True)
    points = np.vstack(chunks).astype(np.float32) if chunks else np.empty((0, 4), np.float32)
    offsets_array = np.array(offsets, dtype=np.int64)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, timestamps=stamps, offsets=offsets_array, points=points)
    return offsets_array, points


def build_scene(config_path: Path) -> Scene:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    dataset = config["dataset"]
    drive = KittiRawDrive(dataset["root"], dataset["date"], dataset["drive"])
    truth = list(drive.ground_truth())
    raw_imu = list(drive.imu())
    degrader = load_degrader_config(config_path)
    window = next(w for w in degrader.windows if w.mode is DegradationMode.BLACKOUT)
    blackout = (window.start_s, window.end_s)
    profile = load_imu_profile(config_path, PROFILE)

    def fuse(seed: int, odometry: list[FrameOdometry] | tuple = ()):
        gnss = list(GnssDegrader(replace(degrader, seed=seed)).degrade(truth))
        imu = list(corrupt_imu(raw_imu, replace(profile.injected, seed=seed)))
        return run_fusion(
            imu,
            truth,
            gnss,
            profile.estimator,
            initial_sigmas=profile.initial_sigmas,
            odometry=odometry,
        )

    def end_error(result) -> float:
        last = [step for step in result.steps if step.timestamp <= blackout[1]][-1]
        return float(np.linalg.norm((last.fused.position_enu_m - last.truth_enu_m)[:2]))

    seed = median_case_seed([end_error(fuse(seed)) for seed in range(SEEDS)])
    print(f"median-case seed: {seed}")

    paths, odometry_by_backend = {}, {}
    end_errors = {}
    for mode, backend, _, _ in MODES:
        odometry: list[FrameOdometry] | tuple = ()
        if backend is not None:
            cache = f"vo_{dataset['date']}_{dataset['drive']}_{backend}.npz"
            noise = VoNoise(**config["visual_odometry"][backend])
            odometry = load_odometry(Path("data/cache") / cache, noise)
            odometry_by_backend[backend] = odometry
        result = fuse(seed, odometry)
        paths[mode] = np.array([step.fused.position_enu_m for step in result.steps])
        end_errors[mode] = end_error(result)
        times = np.array([step.timestamp for step in result.steps])
        truth_at_epochs = np.array([step.truth_enu_m for step in result.steps])
        truth_by_time = result.truth_enu_by_time
    print(f"end of blackout: {end_errors}")

    # Slow down around the start of the blackout and while the errors grow, speed up elsewhere.
    timeline = DemoTimeline(
        DRIVE_START_S,
        [
            Segment(8.0, 18.0),
            Segment(18.0, 30.0),
            Segment(23.0, 32.5),
            Segment(45.0, 65.0),
            Segment(60.0, blackout[1]),
            Segment(68.0, 100.0),
        ],
    )

    frames = _frames_in_span(drive, *timeline.drive_span_s)
    cache = Path("data/cache") / f"demo_matches_{dataset['date']}_{dataset['drive']}.npz"
    offsets, points = _load_matches(cache, frames)
    inliers = {
        record.timestamp: record.constraint.num_inliers
        for record in odometry_by_backend["aliked"]
        if record.constraint is not None
    }
    inliers_by_frame = np.array([inliers.get(frame.timestamp, 0) for frame in frames])

    span = timeline.drive_span_s
    truth_times = np.array(sorted(truth_by_time))
    in_span = truth_times[(truth_times >= span[0]) & (truth_times <= span[1])]
    scene = Scene(
        timeline=timeline,
        blackout=blackout,
        seed=seed,
        times_s=times,
        paths_m=paths,
        truth_at_epochs_m=truth_at_epochs,
        truth_path_m=np.array([truth_by_time[t] for t in in_span]),
        frames=frames,
        match_offsets=offsets,
        match_points=points,
        inliers_by_frame=inliers_by_frame,
    )
    return scene


def _clean_axis(axis) -> None:
    axis.set_facecolor(SURFACE)
    axis.tick_params(colors=TEXT_SECONDARY, length=0, labelsize=11)
    axis.grid(color=GRID, linewidth=0.8)
    axis.set_axisbelow(True)
    for side in ("top", "right", "left", "bottom"):
        axis.spines[side].set_visible(False)


class DriveRenderer:
    """Holds the figure's artists and updates them for each video time."""

    def __init__(self, scene: Scene) -> None:
        self.scene = scene
        self.figure = plt.figure(
            figsize=(WIDTH_PX / DPI, HEIGHT_PX / DPI), dpi=DPI, facecolor=SURFACE
        )
        figure = self.figure
        margin_x, column_gap = 0.02, 0.025
        left_width = 0.64
        right_x = margin_x + left_width + column_gap
        right_width = 1.0 - margin_x - right_x

        self.camera_axis = figure.add_axes([margin_x, 0.635, left_width, 0.343])
        self.camera_axis.set_axis_off()
        first = cv2.imread(str(scene.frames[0].image_path), cv2.IMREAD_GRAYSCALE)
        self.image = self.camera_axis.imshow(first, cmap="gray", vmin=0, vmax=255, aspect="auto")
        self.match_lines = LineCollection([], colors=MATCH_COLOR, linewidths=1.2, alpha=0.9)
        self.camera_axis.add_collection(self.match_lines)
        self.match_points = self.camera_axis.scatter([], [], s=9, color=MATCH_COLOR)
        self.camera_axis.set_xlim(0, first.shape[1])
        self.camera_axis.set_ylim(first.shape[0], 0)
        self.camera_label = self.camera_axis.text(
            0.008,
            0.95,
            "",
            transform=self.camera_axis.transAxes,
            color=TEXT_PRIMARY,
            fontsize=13,
            va="top",
            bbox={"facecolor": SURFACE, "alpha": 0.8, "edgecolor": "none", "pad": 4},
        )

        self._build_map(figure.add_axes([margin_x + 0.028, 0.125, left_width - 0.028, 0.495]))
        self._build_status(figure.add_axes([right_x, 0.855, right_width, 0.123]))
        self._build_readouts(figure.add_axes([right_x, 0.53, right_width, 0.305]))
        self._build_error_chart(figure.add_axes([right_x + 0.038, 0.17, right_width - 0.038, 0.33]))

        self.footer = figure.text(
            0.02,
            0.018,
            f"{ATTRIBUTION}.  Replayed offline, not real time.  Median-case noise seed.",
            color=TEXT_SECONDARY,
            fontsize=11,
            va="center",
        )
        self.speed = figure.text(
            0.98, 0.058, "", color=TEXT_SECONDARY, fontsize=16, va="center", ha="right"
        )

    def _build_map(self, axis) -> None:
        scene = self.scene
        _clean_axis(axis)
        span = scene.timeline.drive_span_s
        in_span = (scene.times_s >= span[0]) & (scene.times_s <= span[1])
        points = np.vstack(
            [scene.truth_path_m[:, :2], *(path[in_span, :2] for path in scene.paths_m.values())]
        )
        center = 0.5 * (points.min(axis=0) + points.max(axis=0))
        half = 0.5 * np.ptp(points, axis=0) + 25.0
        box = axis.get_position()
        aspect = (box.width * WIDTH_PX) / (box.height * HEIGHT_PX)
        half_x = max(half[0], half[1] * aspect)
        half_y = half_x / aspect
        axis.set_xlim(center[0] - half_x, center[0] + half_x)
        axis.set_ylim(center[1] - half_y, center[1] + half_y)
        axis.set_xlabel("East (m)", color=TEXT_SECONDARY, fontsize=12)
        axis.set_ylabel("North (m)", color=TEXT_SECONDARY, fontsize=12)
        axis.plot(
            *scene.truth_path_m[:, :2].T,
            color=TRUTH,
            lw=2,
            ls=(0, (2, 2)),
            alpha=0.7,
            label="Ground truth",
        )
        self.trails, self.heads = {}, {}
        for mode, _, _, color in MODES:
            self.trails[mode] = axis.plot([], [], color=color, lw=2.5)[0]
            self.heads[mode] = axis.plot(
                [], [], "o", ms=10, color=color, mec=SURFACE, mew=2, zorder=5
            )[0]
        axis.legend(frameon=False, labelcolor=TEXT_SECONDARY, loc="upper left", fontsize=12)

    def _build_status(self, axis) -> None:
        axis.set_axis_off()
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        self.status_text = axis.text(
            0.0, 0.5, "", color=TEXT_PRIMARY, fontsize=30, va="center", fontweight="bold"
        )

    def _build_readouts(self, axis) -> None:
        axis.set_axis_off()
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        axis.text(
            0.0,
            0.95,
            "Horizontal error vs ground truth",
            color=TEXT_SECONDARY,
            fontsize=14,
            va="top",
        )
        self.values = {}
        for row, (mode, _, label, color) in enumerate(MODES):
            y = 0.68 - row * 0.29
            axis.plot([0.0, 0.05], [y, y], color=color, lw=5, solid_capstyle="round")
            axis.text(0.08, y, label, color=TEXT_SECONDARY, fontsize=16, va="center")
            self.values[mode] = axis.text(
                1.0,
                y,
                "",
                color=TEXT_PRIMARY,
                fontsize=30,
                va="center",
                ha="right",
                fontweight="bold",
            )

    def _build_error_chart(self, axis) -> None:
        scene = self.scene
        _clean_axis(axis)
        span = scene.timeline.drive_span_s
        axis.axvspan(*scene.blackout, color=BLACKOUT_BAND, zorder=0)
        axis.set_yscale("log")
        axis.set_xlim(*span)
        axis.set_ylim(0.1, 400.0)
        axis.set_xlabel("Drive time (s)", color=TEXT_SECONDARY, fontsize=12)
        axis.set_ylabel("Error (m, log scale)", color=TEXT_SECONDARY, fontsize=12)
        axis.text(
            np.mean(scene.blackout),
            0.96,
            "blackout",
            transform=axis.get_xaxis_transform(),
            ha="center",
            va="top",
            color=TEXT_SECONDARY,
            fontsize=12,
        )
        self.error_lines = {
            mode: axis.plot([], [], color=color, lw=2.2)[0] for mode, _, _, color in MODES
        }

    def _update_camera(self, drive_s: float) -> None:
        scene = self.scene
        stamps = np.array([frame.timestamp for frame in scene.frames])
        index = max(int(np.searchsorted(stamps, drive_s, side="right")) - 1, 0)
        if getattr(self, "_shown_frame", None) != index:
            self._shown_frame = index
            image = cv2.imread(str(scene.frames[index].image_path), cv2.IMREAD_GRAYSCALE)
            self.image.set_data(image)
            start, end = scene.match_offsets[index], scene.match_offsets[index + 1]
            points = scene.match_points[start:end]
            self.match_lines.set_segments(points.reshape(-1, 2, 2))
            self.match_points.set_offsets(points[:, 2:] if len(points) else np.empty((0, 2)))
            self.camera_label.set_text(
                f"Left camera  |  LightGlue matches shown: {len(points)}  |  "
                f"PnP inliers used: {scene.inliers_by_frame[index]}"
            )

    def render(self, video_s: float) -> np.ndarray:
        """Return the RGB frame at this video time."""
        scene = self.scene
        timeline = scene.timeline
        drive_s = timeline.drive_time(video_s)
        self._update_camera(drive_s)
        truth_now = interpolate_path(scene.times_s, scene.truth_at_epochs_m, drive_s)

        past = scene.times_s <= drive_s
        for mode, _, _, _ in MODES:
            path = scene.paths_m[mode]
            current = interpolate_path(scene.times_s, path, drive_s)
            visible = (scene.times_s >= timeline.drive_span_s[0]) & past
            trail = np.vstack([path[visible, :2], current[:2]])
            self.trails[mode].set_data(trail[:, 0], trail[:, 1])
            self.heads[mode].set_data([current[0]], [current[1]])

            error = np.linalg.norm((current - truth_now)[:2])
            self.values[mode].set_text(f"{error:.1f} m")
            offsets = path[visible, :2] - scene.truth_at_epochs_m[visible, :2]
            errors = np.maximum(np.linalg.norm(offsets, axis=1), 0.1)
            self.error_lines[mode].set_data(
                np.r_[scene.times_s[visible], drive_s], np.r_[errors, max(error, 0.1)]
            )

        denied = scene.blackout[0] <= drive_s <= scene.blackout[1]
        self.status_text.set_text("GNSS Unavailable" if denied else "GNSS Available")

        self.speed.set_text(f"Playback ×{timeline.playback_speed(video_s):.1f}")
        self.figure.canvas.draw()
        return np.asarray(self.figure.canvas.buffer_rgba())[:, :, :3].copy()


def _write(writer: cv2.VideoWriter, rgb: np.ndarray) -> None:
    writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def render_video(renderer: DriveRenderer, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"avc1"), FPS, (WIDTH_PX, HEIGHT_PX))
    if not writer.isOpened():
        raise SystemExit("could not open an H.264 video writer")
    frame_count = int(renderer.scene.timeline.duration_s * FPS)
    for index in range(frame_count):
        _write(writer, renderer.render(index / FPS))
        if (index + 1) % 300 == 0:
            print(f"  video: {index + 1}/{frame_count} frames", flush=True)
    writer.release()


def _gif_palette() -> Image.Image:
    """A fixed palette: exact interface colours, plus a gray ramp for the camera.

    Quantising each frame on its own lets a thin blue line collapse into the nearest green, which
    would make the three modes impossible to tell apart.
    """
    named = [mode[3] for mode in MODES]
    named += [TEXT_PRIMARY, TEXT_SECONDARY, SURFACE, GRID, BLACKOUT_BAND]
    colors = [tuple(int(value[i : i + 2], 16) for i in (1, 3, 5)) for value in named]
    colors += [(level, level, level) for level in np.linspace(0, 255, 48).astype(int).tolist()]
    flat = [channel for color in colors for channel in color]
    palette = Image.new("P", (1, 1))
    palette.putpalette(flat + [0] * (768 - len(flat)))
    return palette


def render_gif(renderer: DriveRenderer, path: Path, start_s=18.0, end_s=62.0, seconds=14.0) -> None:
    """A short, small loop of the blackout for the top of the README."""
    fps = 10
    palette = _gif_palette()
    images = []
    for video_s in np.linspace(start_s, end_s, int(seconds * fps)):
        small = cv2.resize(
            renderer.render(float(video_s)), (720, 405), interpolation=cv2.INTER_AREA
        )
        images.append(Image.fromarray(small).quantize(palette=palette, dither=Image.Dither.NONE))
    path.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(path, save_all=True, append_images=images[1:], duration=1000 // fps, loop=0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=Path("configs/default_sim.yaml"))
    parser.add_argument("--output", type=Path, default=Path("assets/demo.mp4"))
    parser.add_argument("--gif", type=Path, help="also write a short loop for the README")
    parser.add_argument("--gif-only", action="store_true", help="skip the full video")
    parser.add_argument("--stills", type=float, nargs="*", help="video times to save as PNG, only")
    parser.add_argument("--stills-dir", type=Path, default=Path("assets/stills"))
    args = parser.parse_args()

    renderer = DriveRenderer(build_scene(args.config))

    if args.stills is not None:
        args.stills_dir.mkdir(parents=True, exist_ok=True)
        for video_s in args.stills:
            Image.fromarray(renderer.render(video_s)).save(
                args.stills_dir / f"t{video_s:05.1f}.png"
            )
        print(f"stills: {args.stills_dir}")
        return 0

    if not args.gif_only:
        render_video(renderer, args.output)
        print(f"video: {args.output}")
    if args.gif is not None:
        render_gif(renderer, args.gif)
        print(f"gif: {args.gif}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
