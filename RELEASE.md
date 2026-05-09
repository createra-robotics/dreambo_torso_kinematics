# Releasing

## What triggers a release

Pushing **any git tag** to `origin` is the single trigger that publishes new packages.

Specifically, a tag push runs the `Python Bindings CI` workflow (`.github/workflows/python-bindings.yml`), which is configured with:

```yaml
on:
  push:
    tags:
      - '*'
```

That workflow builds wheels for Linux (glibc + musl, x86_64/x86/aarch64/armv7), Windows (x64/x86), macOS (aarch64), and an sdist, then runs the `release` job which:

- Generates artifact attestations for every wheel.
- Publishes the wheels and sdist to **PyPI** via `maturin upload` (the `Publish to PyPI` step, gated on `startsWith(github.ref, 'refs/tags/')`).

> Note: the **crates.io** publish is **not currently automated** by tag pushes. To update the crate, run `cargo publish` manually after the tag is pushed, or add a `cargo publish` step to the workflow gated on `startsWith(github.ref, 'refs/tags/')`.

The `Rust` workflow (`.github/workflows/rust.yml`) only runs on pull requests for build/test/fmt checks and does not publish anything.

## Release a new version

Bump the version in `Cargo.toml` (and let it propagate to `pyproject.toml` via maturin) and commit, then tag and push:

```bash
git commit -am "Release v1.0.1"
git push
git tag -a v1.0.1 -m "Release v1.0.1"
git push origin v1.0.1

# or
git commit -am "Release v1.0.1"
git tag -a v1.0.1 -m "Release v1.0.1"
git push --follow-tags
```

The tag push is what fires the wheel build and PyPI upload.

## Redo a version

If a tagged build failed or the version needs to be re-cut against a new commit, delete the remote tag and re-push it:

```bash
git push origin :refs/tags/v1.0.0
git push origin v1.0.0
```

Re-pushing the tag re-triggers the same workflow. PyPI upload uses `--skip-existing`, so any wheels already on PyPI for that version will be skipped — bump the version if you need fresh artifacts to actually replace the published ones.