# Contributing

Contributions, bug reports, and focused pull requests are welcome.

## Development checks

Build the firmware with ESP-IDF 5.5.x:

```sh
idf.py -C firmware/sticks3 build
```

## Privacy gate

Treat every commit as public. Before pushing:

1. Review the working tree, staged diff, commit metadata, and every untracked
   file.
2. Do not commit credentials, Wi-Fi details, private email addresses, local
   absolute paths, device identifiers, logs, recordings, or raw service data.
3. Inspect generated binaries and image metadata.
4. Use a GitHub `noreply` address if you do not want a personal email exposed
   in commit metadata.
5. Do not use `git add .` until every untracked file has been reviewed.

Keep changes focused and document user-visible behavior. By contributing, you
agree that your contribution is licensed under the repository's MIT License.
