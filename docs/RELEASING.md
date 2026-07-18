# Releasing to PyPI — step-by-step

Publishing uses **trusted publishing** (GitHub OIDC): PyPI trusts this repo's
`release.yml` workflow directly. No API tokens to create, store, or leak.

## One-time setup (~10 minutes, in a browser)

You need a [pypi.org](https://pypi.org) account (and a
[test.pypi.org](https://test.pypi.org) account — separate signup) with 2FA
enabled.

On **both** sites, register a *pending publisher* (pending = the project name
doesn't exist yet; publishing creates and claims it):

1. Log in → click your avatar → **Your account** → **Publishing**.
2. Under "Add a new pending publisher" (GitHub tab), enter exactly:

   | Field | Value |
   | --- | --- |
   | PyPI Project Name | `lerobot_robot_fastgripper` |
   | Owner | `Transcend-Mechanics` |
   | Repository name | `fastgripper-lerobot` |
   | Workflow name | `release.yml` |
   | Environment name | `pypi` (use `testpypi` on test.pypi.org) |

3. In the GitHub repo: **Settings → Environments** → create two environments
   named `pypi` and `testpypi` (no secrets needed; optionally add yourself as
   a required reviewer on `pypi` so releases need a click of approval).

## Routine packaging health (automatic)

CI's `package-smoke` job already builds the wheel, installs it in a clean
venv, and checks plugin discovery + the CLI on **every push** — so packaging
breakage is caught continuously without publishing anything. TestPyPI is a
*rehearsal of the publish step itself* (OIDC auth, upload), best used right
before a real release rather than on a schedule: it permanently rejects
re-uploads of the same version, so automating it would need artificial
version churn for no extra signal.

## Dry run against TestPyPI (recommended before the first real release)

1. GitHub → **Actions → Release → Run workflow** (leave branch = main).
   This builds and publishes the current `0.1.0.dev0` to TestPyPI.
2. Verify the artifact installs and is discoverable, in a clean venv:

   ```sh
   uv venv /tmp/fgtest --python 3.12
   uv pip install -p /tmp/fgtest/bin/python "lerobot[feetech]>=0.6.0,<0.7"
   uv pip install -p /tmp/fgtest/bin/python \
     --index-url https://test.pypi.org/simple/ --no-deps lerobot_robot_fastgripper
   /tmp/fgtest/bin/python -c "
   from lerobot.utils.import_utils import register_third_party_plugins
   from lerobot.robots.config import RobotConfig
   register_third_party_plugins()
   print(RobotConfig.get_choice_class('fastgripper_follower'))"
   /tmp/fgtest/bin/fastgripper --help
   ```

## The real release

1. Bump the version in `pyproject.toml` (e.g. `0.1.0.dev0` → `0.1.0`),
   commit, push, and wait for CI to go green.
2. Tag and push the tag — this *is* the release trigger:

   ```sh
   git tag v0.1.0
   git push origin v0.1.0
   ```

3. Watch **Actions → Release**. When `publish-pypi` finishes,
   `pip install lerobot_robot_fastgripper` is live worldwide (and the name
   is permanently claimed).
4. Optionally create a GitHub Release from the tag with notes:
   `gh release create v0.1.0 --generate-notes`.

## Future releases

Bump version → commit → tag `vX.Y.Z` → push tag. That's the whole process.

Guidelines:
- Bump the **lerobot pin** (`>=0.6.0,<0.7`) only after the weekly
  lerobot@main CI canary has passed against the new upstream version.
- Note any change to hardware-validated constants (homing thresholds,
  velocity/torque profile) in the release notes — users' grippers depend on
  them.
