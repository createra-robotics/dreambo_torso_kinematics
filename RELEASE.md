

## Release a new version

```bash
git tag -a v1.0.1 -m "Release v1.0.1"
git push origin v1.0.1
```

## Redo a version
```bash
git push origin :refs/tags/v1.0.0
git push origin v1.0.0
```