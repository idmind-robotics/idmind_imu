# Contributing to idmind_imu

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

## Working on this package

This package is developed by [IDMind](https://idmind.pt/) and consumed elsewhere as a git
submodule, so keep every commit scoped to `idmind_imu`.

- Branch off `master` and open a pull request against it.
- Use [Conventional Commits](https://www.conventionalcommits.org/): `type(scope): summary`
  (`feat`, `fix`, `docs`, `refactor`, `chore`, `test`).
- New source files must carry the standard Apache 2.0 header.

Before opening a pull request, both the build and the full test suite must pass:

```bash
colcon build --symlink-install --packages-select idmind_imu
colcon test --packages-select idmind_imu && colcon test-result --all --verbose
```

The suite runs gtest plus `ament_lint_auto` (cpplint, uncrustify, lint_cmake, xmllint, flake8,
copyright); max line length is 100. See `CLAUDE.md` for the architecture notes and the
invariants that must not be "simplified" away.
