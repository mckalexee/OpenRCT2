# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OpenRCT2 is an open-source re-implementation of RollerCoaster Tycoon 2 in C++20. It's a cross-platform amusement park simulation game supporting Windows, Linux, macOS, and Android.

## Build Commands

### Windows (MSBuild)

Uses the Visual Studio solution file. Dependencies are downloaded automatically on first build.

**IMPORTANT:** Use PowerShell (not cmd.exe) for build commands. MSBuild is in the PATH in PowerShell.

**Build:**
```
powershell.exe -Command "msbuild openrct2.sln /p:Configuration=Release /p:Platform=x64 /m /v:minimal"
```

**Build configurations:** `Debug`, `Release`, `ReleaseLTCG`
**Platforms:** `x64`, `Win32`, `arm64`

**Output:** `bin\openrct2.exe`, `bin\tests.exe`

**Run tests (from bin directory):**
```
powershell.exe -Command "cd 'N:\src\OpenRCT2\bin'; .\tests.exe --gtest_filter=*TestName*"
```

### Linux/macOS (CMake)

Out-of-source build required.

**Build:**
```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

**Build with tests:**
```bash
cmake -DWITH_TESTS=ON ..
cmake --build .
```

**Run tests (from build directory):**
```bash
ctest -j 2 --output-on-failure
```

**Check code formatting:**
```bash
scripts/run-clang-format.py -i --clang-format-executable clang-format -r src test data/shaders --exclude src/thirdparty --extensions "c,h,cpp,hpp,cc,hh,cxx,hxx,vert,frag,mm"
git diff --exit-code
```

**Key CMake options:**
- `WITH_TESTS` - Build test suite
- `ENABLE_SCRIPTING` - JavaScript plugin support (default ON)
- `DISABLE_NETWORK` - Disable multiplayer
- `DISABLE_GUI` - Headless mode
- `PORTABLE` - Portable build with relative paths

## Architecture

### Core Pattern: Service Locator
The `IContext` interface (`src/openrct2/Context.h`) is the main service locator providing access to all subsystems:
- `GetAudioContext()`, `GetUiContext()`, `GetObjectManager()`, `GetScriptEngine()`, `GetNetwork()`, `GetDrawingEngine()`

### Game Loop
- Game update rate: 40 FPS (25ms per tick)
- Network update rate: 140 FPS
- Multiple scenes: Preloader, Intro, Title, Game, Editor

### Key Source Directories (`src/openrct2/`)

| Directory | Purpose |
|-----------|---------|
| `actions/` | Player action/command system (replicated in multiplayer) |
| `entity/` | Game entities (peeps, vehicles) |
| `ride/` | Ride data, track, vehicles, physics |
| `peep/` | Guest/staff AI, pathfinding |
| `world/` | Map, climate, footpaths, scenery |
| `object/` | Rides, shops, scenery object management |
| `interface/` | Window/widget system |
| `scripting/` | JavaScript/Duktape plugin system |
| `network/` | Multiplayer, network sync |
| `drawing/` | Graphics, palette, drawing primitives |
| `rct1/`, `rct2/`, `rct12/` | Vanilla RCT compatibility layers |

### Key Entry Points
- `OpenRCT2.cpp` - Main entry
- `Context.cpp` - Core context implementation
- `Game.cpp` - Game loop and state

## Code Style

- **C++20** with GCC 12+ or modern Clang/MSVC
- **Indentation:** 4 spaces, no tabs
- **Line length:** 128 columns
- **Formatting:** WebKit-based clang-format (see `.clang-format`)
- **Pointers:** Left-aligned (`int* ptr`)

## Adding New Strings

1. Add string entry to `data/language/en-GB.txt` (only edit en-GB in this repo)
2. Add string constant to `src/openrct2/localisation/StringIds.h`
3. Create issue in OpenRCT2/Localisation repo to notify translators

## Git Workflow

- Uses **gitflow**: branch from `develop`, never from `master`
- `master` is only for tagged releases
- PRs go to `develop` branch

## Testing

- Uses Google Test framework
- Tests in `test/tests/`
- Test data in `test/tests/testdata/`
- Windows: Tests are built as part of the solution (always included)
- Linux/macOS: Requires `WITH_TESTS=ON` CMake option
