## Public and Private Namespace

The namespaces of the libVDA5050++ are divided into a public [`vda5050pp::`](../doxygen/html/namespacevda5050pp.html)
and a private [`vda5050pp::core`](../doxygen/html/namespacevda5050pp_1_1core.html)
namespace.
The respective headers are in the `<root>/include/public` and `<root>/include/private` directories.
Private dependencies of the library should only be exposed in the private headers. Notably the [NLohmann-JSON](../dependencies.md#nlohmann-json-automatic) dependency, since it is tightly related with the VDA5050 data structures.

## Directories

For each header file, let's say `include/private/vda5050++/core/action_event_manager.h`, there is a corresponding
source file `src/vda5050++/core/action_event_manager.cpp`. There is no [private/public](#public-and-private-namespace) separation
for the source files. Each source file also usually has one test file `test/vda5050++/core/action_event_manager.cpp`.

The overall directory structure looks like this:
```
<root>
├── cmake -> Auxiliary cmake files
├── docker -> Docker files for containerized builds
├── docs -> The contents of this documentation
├── examples -> Example libVDA5050++ integration
│   └── src
├── include
│   ├── private -> Private includes of the libVDA5050++ (the core namespace)
│   │   └── vda5050++
│   │       └── core
│   └── public -> Public includes of the libVDA5050++
│       └── vda5050++
├── patches -> patches used for third party libraries
├── src -> the main sources of the libVDA5050++
│   └── vda5050++
│       └── core
└── test
    ├── include -> include files for tests only
    │   └── test
    ├── src -> sources for common test code
    └── vda5050++ -> Test files roughly resembling the directory structure inside of src/vda5050++
```

## Naming convention

- **Member variables** should be in `lower_snake_case_` with a trailing "_"
- **Other variables** and **parameters** should be in `lower_snake_case`
- **Class and Struct** names should be in `UpperCamelCase`
- **Member functions** should be in `lowerCamelCase`
- **`using namespace *`** may only be used inside of a single [Translation Unit](https://en.wikipedia.org/wiki/Translation_unit_(programming)) (i.e. not in header files)
- **Source files** should end in `.cpp`
- **Header files** should end in `.h`
- **Constants** should be prefixed with `k_`. This includes `enum` enumerators.


## Git commit messages

- Follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) scheme
- Messages should be _imperative_. E.g. **fix a bug**, **add a feature**, ... So in general imagine the commit message
  in a sentence like _if applied, this commit will ..._


## Tools

The tools affecting the code base currently are ran by [pre-commit](https://pre-commit.com/).
Pre-commit is a tool that creates [git-hooks](https://git-scm.com/book/en/v2/Customizing-Git-Git-Hooks) that
run before each commit.  In this project the [clang-format](https://clang.llvm.org/docs/ClangFormat.html)
formatter is run to ensure consistent code formatting and conventional commit rules are enforced.

To install the hooks, do:

```sh
apt install pre-commit
cd libvda5050++
pre-commit install && pre-commit install --hook-type commit-msg
```

## CPM and it's cache

The dependencies are managed with [CPM](https://github.com/cpm-cmake/CPM.cmake).
By default CPM downloads the dependencies into the build directory, however this may be slow when
you casually clean up your build directories.  You can (and should) set your cache path for CPM
with :
```sh
export CPM_SOURCE_CACHE="$HOME/.cache/CPM" # or some other directory
```
If you do not set the cache path, be aware, that there are currently issues regarding
patches applied to sources (which has to be done for paho) see [this PR](https://github.com/cpm-cmake/CPM.cmake/pull/580).


## Branches versions and merge requests

Originally there was the `main` branch with the production code and the
`dev` branch with staging production code. `main` is protected and `dev` should be protected, too.
However gitlab mirroring automatically mirrors all protected branches, so there was no other way but to
leave `dev` unprotected.

In order to add a new patch/feature/... create a MR with `dev` as it's target. MR's should always be **squashed**!
To publish a bunch of commits new to `dev`, a new version according to [semver](https://semver.org/)
is chosen. _Note, that major version 3 is for VDA5050-2.1.0 and major version 2 is for VDA5050-2.0.0._ 
For each version update an extra commit with a message, such as
```
chore: :bookmark: set version to x.x.x
```
changing the version number in the top-level `CMakeLists.txt` is done.
Then `main` is fast-forwarded to `dev` and a tag is assigned to the version number update commit.

Since both VDA5050 2.1.0 and 2.0.0 are maintained, there also exists a `VDA5050-2.0.0` and a `VDA5050-2.0.0-dev`
branch with analogous purposes as `main` and `dev` but for the 2.0.0 version.
Fixes that would apply for both branches, can be [cherry-picked](https://git-scm.com/docs/git-cherry-pick).