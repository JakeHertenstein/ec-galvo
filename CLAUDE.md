# ec-galvo

Public, MIT. Driver only: no Epoch Cipher appliance code, card data, layouts,
bundles or machine files belong here. Those live in the private `burner` repo.

- Never log coordinates or packet contents. `tests/test_controller.py` checks this.
- No silent fallbacks: out-of-range moves, bad .cor files and USB errors raise.
- Keep `UPSTREAM.md` current when pulling more code from MeerK40t.
- Checks: `ruff check ec_galvo tests && ruff format --check ec_galvo tests && pytest`.
