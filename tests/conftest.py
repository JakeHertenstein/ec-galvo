"""Fail loudly if the tests would run against an installed ec-galvo instead of this
checkout (burner's dev shell has a pinned copy installed)."""

import pathlib

import ec_galvo

_REPO = pathlib.Path(__file__).resolve().parent.parent
_LOADED = pathlib.Path(ec_galvo.__file__).resolve()

if not _LOADED.is_relative_to(_REPO):
    raise RuntimeError(
        f"tests imported ec_galvo from {_LOADED}, not from this checkout ({_REPO}); "
        "run pytest from the repo folder so pyproject's pythonpath applies"
    )
