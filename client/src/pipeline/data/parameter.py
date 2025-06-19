from dataclasses import dataclass
from pathlib import Path

from rich.console import Console
from rich.progress import Progress


from ..config import INPUT_PATH
from .utils import (
    ExperimentBundle,
    LoadedExperiment,
    is_associated_experiment,
)

LOADED_DIR = "loaded"
UNLOADED_DIR = "unloaded"


class AssociationError(Exception):
    def __init__(
        self,
        experiment: LoadedExperiment,
        candidates: list[LoadedExperiment],
    ) -> None:
        dir_name = experiment.path.parent.parent.name
        msg = f"No candidates found for experiment {experiment.path.name} ({dir_name})"

        if candidates:
            msg = f"Multiple candidates found for experiment {experiment.path.name} ({dir_name}):"

            msg += f"{', '.join(f" - {c.path.name}" for c in candidates)}"

        super().__init__(msg)

        self.experiment = experiment
        self.candidates = candidates


@dataclass
class Dataset:
    path: Path
    bundles: list[ExperimentBundle]
    errors: list[AssociationError] | None = None


class DatasetLoader:

    def __init__(self, path: Path, console: Console) -> None:
        self.console = console
        self.path = path

        if not self.path.exists():
            raise FileNotFoundError(f"Dataset path does not exist: {self.path}")

    def load(self) -> Dataset:
        self.console.print("> Unloaded...")
        loaded = self._load_experiments(LOADED_DIR)

        if loaded is None:
            raise FileNotFoundError(f"Could not find loaded experiments in {self.path}")

        loaded_primary, loaded_calibration = self._split_bundles_wind(loaded)
        (loaded_bundles, errors) = self._associate_bundles(
            loaded_primary, loaded_calibration
        )

        if errors:
            self._print_error("loaded experiments", errors)

        self.console.print("> Unloaded...")
        unloaded = self._load_experiments(UNLOADED_DIR)

        if not unloaded:
            return Dataset(
                path=self.path,
                bundles=loaded,
                errors=None,
            )

        unloaded_primary, unloaded_calibration = self._split_bundles_wind(unloaded)
        (unloaded_bundles, unloaded_errors) = self._associate_bundles(
            unloaded_primary, unloaded_calibration
        )

        if unloaded_errors:
            self._print_error("unloaded experiments", unloaded_errors)

        self.console.print("> Combining loaded and unloaded experiments...")
        (bundles, errors) = self._associate_bundles(loaded_bundles, unloaded_bundles)

        if errors:
            self._print_error("combined experiments", errors)

        return Dataset(
            path=self.path,
            bundles=bundles,
            errors=errors if errors else None,
        )

    def _load_experiments(self, subdirectory: str) -> list[ExperimentBundle] | None:
        subpath = self.path / subdirectory

        if not subpath.exists():
            return None

        return [
            ExperimentBundle(LoadedExperiment(file))
            for file in subpath.glob("*.parquet")
            if file.is_file()
        ]

    def _split_bundles_wind(
        self, bundles: list[ExperimentBundle]
    ) -> tuple[list[ExperimentBundle], list[ExperimentBundle]]:
        primaries = []
        candidates = []

        for bundle in bundles:
            assert bundle.primary.parsed is not None

            if bundle.primary.parsed.parameters["w"] > 0:
                primaries.append(bundle)

            else:
                candidates.append(bundle)

        return (primaries, candidates)

    def _associate_bundles(
        self,
        primaries: list[ExperimentBundle],
        candidates: list[ExperimentBundle],
    ) -> tuple[list[ExperimentBundle], list[AssociationError]]:

        associations = []
        errors = []

        for primary in primaries:
            try:
                associated = self._associate_bundle(primary, candidates)
                associations.append(associated)

            except AssociationError as e:
                errors.append(e)

        return (associations, errors)

    def _associate_bundle(
        self,
        bundle: ExperimentBundle,
        candidates: list[ExperimentBundle],
    ) -> ExperimentBundle:
        self.console.print(
            f"[cyan]Checking {bundle.primary.path.name} ({len(candidates)} candidates)[/]"
        )

        for c in candidates:
            associated = is_associated_experiment(
                bundle.primary.get_parsed(),
                c.primary.get_parsed(),
            )

            self.console.print(
                f"  - {c.primary.path.name}: {'associated' if associated else 'not associated'}"
            )

        associated = [
            c
            for c in candidates
            if is_associated_experiment(
                bundle.primary.get_parsed(), c.primary.get_parsed()
            )
        ]

        match len(associated):
            case 0:
                raise AssociationError(bundle.primary, [])

            case 1:
                return bundle - associated[0]

            case _:
                raise AssociationError(bundle.primary, [a.primary for a in associated])

    def _print_error(self, what: str, errors: list[AssociationError]) -> None:
        self.console.print(
            f"[red]Errors while associating {what} in {self.path.name}[/]"
        )

        for error in errors:
            self.console.print(f"[red]  {error}[/]")


def load_unprocessed_datasets(progress: Progress) -> list[Dataset]:
    if not INPUT_PATH.exists():
        raise FileNotFoundError(
            f"Unprocessed data directory does not exist: {INPUT_PATH}"
        )

    task = progress.add_task("Loading datasets", total=None)

    paths = [p for p in INPUT_PATH.iterdir() if p.is_dir()]
    progress.update(task, total=len(paths))

    datasets = []

    for dataset in paths:
        if dataset.name != "free-flight":
            continue

        loader = DatasetLoader(dataset, progress.console)
        datasets.append(loader.load())
        progress.advance(task)

    return datasets
