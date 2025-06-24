from pathlib import Path
from typing import Protocol

from carac.helpers import Vec3
from pandas import DataFrame

from ..config import INPUT_PATH
from ..utils import WindLut
from .changes import (
    actuate_from_filename,
    add_idle_actuation,
    correct_load_orientation,
    patch_robot_orientation,
    remap_sweep,
    remap_throttle,
    undo_sweep_l_fix,
)
from .utils import (
    ExperimentBundle,
    LoadedExperiment,
    chunk_list,
    combine_calibrated_bundles,
    combine_strided_bundles,
)

STANDARD_OFFSET: Vec3 = (-1.12684e-02, -1.05960e-03, 1.85208e-01)
LUT_46 = {0.46: 5.0}
LUT_30_50 = {0.3: 4.1, 0.5: 5.8}
LUT_48_50 = {0.49: 5.7, 0.5: 5.8}
LUT_20_40_60 = {0.2: 2.3, 0.4: 5.2, 0.6: 6.5}


# == Base class == #


class Loader(Protocol):

    name: str
    wind: WindLut
    offset: Vec3

    def __init__(
        self,
        name: str,
        wind: WindLut,
        offset: Vec3 = STANDARD_OFFSET,
    ) -> None:
        self.name = name
        self.offset = offset
        self.wind = wind

    def load_bundles(self) -> list[ExperimentBundle]: ...

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        pass

    def postprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        pass

    @staticmethod
    def read_experiments(path: Path) -> list[ExperimentBundle]:
        """Read a list of experiment parquet's from a given path."""

        experiments = [
            ExperimentBundle(LoadedExperiment(file))
            for file in path.iterdir()
            if file.name[0:4].isnumeric() and file.suffix == ".parquet"
        ]

        if not experiments:
            raise ValueError(
                f"No experiments found in {path}. Ensure the path is correct and contains parquet files."
            )

        experiments.sort(key=lambda x: x.primary.path.name)
        return experiments


# == Dataset Loaders == #


class PlungeLoader(Loader):
    def __init__(self):
        super().__init__(name="plunge", wind=WindLut(LUT_46))
        self.path = INPUT_PATH / self.name

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")
        unloaded = self.read_experiments(self.path / "unloaded")

        bundles = combine_calibrated_bundles(
            positive_bundles=combine_strided_bundles(loaded, 8),
            negative_bundles=combine_strided_bundles(unloaded, 8),
        )

        assert len(bundles) == 8
        return bundles

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        del experiment
        patch_robot_orientation(df)
        correct_load_orientation(df)
        add_idle_actuation(df)


class UncoupledAxisLoader(Loader):
    def __init__(self):
        super().__init__(name="axis-uncoupled", wind=WindLut(LUT_30_50))
        self.path = INPUT_PATH / "axis"

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")[0:486]
        unloaded = self.read_experiments(self.path / "unloaded")[0:18]

        unloaded_calibrated = combine_strided_bundles(unloaded, 6)

        bundles = [
            composition
            for chunk in chunk_list(loaded, 18)
            for composition in combine_calibrated_bundles(
                positive_bundles=combine_strided_bundles(chunk, 6),
                negative_bundles=unloaded_calibrated,
            )
        ]

        assert len(bundles) == 324
        return bundles

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        patch_robot_orientation(df)
        actuate_from_filename(df, experiment)
        undo_sweep_l_fix(df)
        remap_sweep(df)
        remap_throttle(df)


class CoupledAxisLoader(Loader):
    def __init__(self):
        super().__init__(name="axis-coupled", wind=WindLut(LUT_30_50))
        self.path = INPUT_PATH / "axis"

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")[486:558]
        unloaded = self.read_experiments(self.path / "unloaded")[18:26]

        loaded_calibrated = combine_strided_bundles(loaded, 36)
        unloaded_calibrated = combine_strided_bundles(unloaded, 4)

        bundles = combine_calibrated_bundles(
            positive_bundles=loaded_calibrated,
            negative_bundles=unloaded_calibrated,
        )

        assert len(bundles) == 36
        return bundles

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        patch_robot_orientation(df)
        actuate_from_filename(df, experiment)
        undo_sweep_l_fix(df)
        remap_sweep(df)
        remap_throttle(df)


class FreeFlightLoader(Loader):
    def __init__(self):
        super().__init__(name="free-flight", wind=WindLut(LUT_30_50))
        self.path = INPUT_PATH / self.name

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")
        unloaded = self.read_experiments(self.path / "unloaded")

        loaded_symmetric = loaded[:8]
        loaded_asymmetric = loaded[8:20]
        loaded_throttle = loaded[20:22]

        symmetric_calibrated = combine_strided_bundles(loaded_symmetric, 4)
        throttle_calibrated = combine_strided_bundles(loaded_throttle, 1)

        # The asymmetric runs were recorded in an incorrect order.
        # The first 6 runs are the "positive" set and the last 6 are the "negative" set
        asymmetric_calibrated = combine_calibrated_bundles(
            positive_bundles=loaded_asymmetric[:6],
            negative_bundles=loaded_asymmetric[6:],
        )

        unloaded_calibrated = combine_calibrated_bundles(
            positive_bundles=unloaded[1:2], negative_bundles=unloaded[0:1]
        )

        assert len(symmetric_calibrated) == 4
        assert len(asymmetric_calibrated) == 6
        assert len(throttle_calibrated) == 1
        assert len(unloaded_calibrated) == 1

        return combine_calibrated_bundles(
            positive_bundles=(
                symmetric_calibrated + asymmetric_calibrated + throttle_calibrated
            ),
            negative_bundles=unloaded_calibrated,
        )

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        del experiment
        patch_robot_orientation(df)
        remap_sweep(df)
        remap_throttle(df)


class FreeFlightExtendedLoader(Loader):
    def __init__(self):
        super().__init__(name="free-flight-extended", wind=WindLut(LUT_30_50))
        self.path = INPUT_PATH / self.name

    def load_bundles(self) -> list[ExperimentBundle]:
        files = self.read_experiments(self.path)

        calibration = files.pop()

        return combine_calibrated_bundles(
            positive_bundles=files,
            negative_bundles=[calibration],
        )

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        del experiment
        patch_robot_orientation(df)
        remap_sweep(df)
        remap_throttle(df)


class FreeFlight2Loader(Loader):
    def __init__(self):
        super().__init__(name="free-flight-2", wind=WindLut(LUT_48_50))
        self.path = INPUT_PATH / self.name

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")
        unloaded = self.read_experiments(self.path / "unloaded")

        loaded_calibrated = combine_strided_bundles(loaded[0:4], 2)
        unloaded_calibrated = combine_strided_bundles(unloaded, 1)

        validation_loaded_calibrated = combine_strided_bundles(loaded[4:6], 1)

        assert len(loaded_calibrated) == 2
        assert len(validation_loaded_calibrated) == 1
        assert len(unloaded_calibrated) == 1

        return [
            *combine_calibrated_bundles(
                positive_bundles=[*loaded_calibrated, *validation_loaded_calibrated],
                negative_bundles=unloaded_calibrated,
            ),
        ]

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        del experiment
        patch_robot_orientation(df)
        remap_sweep(df)
        remap_throttle(df)


class AttackRotationLoader(Loader):
    def __init__(self):
        super().__init__(name="attack-rotation", wind=WindLut(LUT_20_40_60))
        self.path = INPUT_PATH / self.name

    def load_bundles(self) -> list[ExperimentBundle]:
        loaded = self.read_experiments(self.path / "loaded")
        unloaded = self.read_experiments(self.path / "unloaded")

        loaded_calibrated = combine_strided_bundles(loaded, 8)
        unloaded_calibrated = combine_strided_bundles(unloaded, 8)

        bundles = combine_calibrated_bundles(
            positive_bundles=loaded_calibrated,
            negative_bundles=unloaded_calibrated,
        )

        assert len(bundles) == 24
        return bundles

    def preprocess(self, df: DataFrame, experiment: LoadedExperiment) -> None:
        del experiment
        patch_robot_orientation(df)
        remap_sweep(df)
        remap_throttle(df)


LOADERS: list[Loader] = [
    PlungeLoader(),
    UncoupledAxisLoader(),
    CoupledAxisLoader(),
    FreeFlightLoader(),
    FreeFlightExtendedLoader(),
    FreeFlight2Loader(),
    AttackRotationLoader(),
]
