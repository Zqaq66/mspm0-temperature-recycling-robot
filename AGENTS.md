# Repository Maintenance

- Work on `main` unless a task requires a separate branch.
- Check `git status` before editing and preserve unrelated local changes.
- After completing and verifying a development task, commit the scoped changes with a clear message and push `main` to `origin`.
- Do not commit build outputs, Keil user state, duplicate PCB exports, secrets, credentials, or private machine configuration.
- Preserve the repository's existing file encoding and line endings as defined by `.gitattributes`.
- Do not force-push, rewrite published history, create releases, change repository visibility, or publish externally without explicit user confirmation.
- If verification fails or hardware evidence is required, report the limitation instead of presenting the task as fully verified.
