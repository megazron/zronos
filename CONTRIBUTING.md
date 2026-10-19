# Contributing to zronOS

```bash
make && make test          # build + full suite
make san && bash tests/run_tests.sh
pip install -r bridge/requirements.txt && python -m unittest discover -s tests -p 'test_*.py'
```

Language changes belong in [Robot Chalao](https://github.com/megazron/robot-chalao) first. Then run `tools/sync-chalao.sh <tag>` here; CI checks `vendor/chalao` against `vendor/chalao/VERSION`.

Release: bump `OS_VERSION` in `src/zron.cpp` and `tests/run_tests.sh`, update `CHANGELOG.md`, then push a `vX.Y.Z` tag.
