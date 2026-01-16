# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OpenRCT2 is an open-source re-implementation of RollerCoaster Tycoon 2 written in C++20. The project aims to provide everything from RCT2 plus improvements like modern platform support, multiplayer, improved AI, and plugin scripting.

## Build Commands

### macOS (using CMake + Ninja)
```bash
mkdir -p bin && cd bin
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DWITH_TESTS=on
ninja
```

The macOS build automatically downloads dependencies to `lib/macos/` and creates `OpenRCT2.app`.

### Linux
```bash
mkdir -p bin && cd bin
cmake .. -G Ninja -DCMAKE_INSTALL_PREFIX=/usr -DWITH_TESTS=on
ninja all install
```

### Running Tests
```bash
cd bin
./openrct2-cli scan-objects  # Build object indexes first
ctest -j 2 --output-on-failure
```

Or run tests directly: `./OpenRCT2Tests`

### Code Formatting
The project uses clang-format (version 20). Format code with:
```bash
scripts/run-clang-format.py -i -r src test --exclude src/thirdparty
```

## Architecture

### Build Targets
- **libopenrct2**: Core game library (src/openrct2/)
- **openrct2**: GUI application using SDL2 (src/openrct2-ui/)
- **openrct2-cli**: Headless CLI for servers/tools (src/openrct2-cli/)
- **OpenRCT2Tests**: Google Test suite (test/tests/)

### Source Structure (src/openrct2/)
- **actions/**: Game actions - player commands that modify game state. Each action has Query (validation) and Execute phases for multiplayer sync.
- **audio/**: Sound mixing and music playback
- **core/**: Utilities: strings, compression, encryption, serialization
- **drawing/**: Low-level sprite rendering and palette handling
- **entity/**: Game entities (guests, staff, vehicles, etc.)
- **interface/**: Window/widget system and input handling
- **localisation/**: String IDs, translations, currency, dates
- **management/**: Park finances, marketing, research
- **network/**: Multiplayer networking and game synchronization
- **object/**: Game objects (rides, scenery, shops) and their loading
- **paint/**: Prepares sprites for drawing, handles visibility
- **peep/**: Guest and staff AI, pathfinding, actions
- **ride/**: Ride mechanics, vehicle physics, track data
- **rct1/, rct2/, rct12/**: Original game format compatibility
- **scenario/**: Scenario loading and objectives
- **scripting/**: JavaScript plugin engine (duktape-based)
- **world/**: Map, tiles, climate, park data

### Key Patterns

**Game Actions**: All game state changes go through GameAction classes in `actions/`. Each action implements:
- `Query()`: Validates if action is allowed (checks money, permissions, collisions)
- `Execute()`: Performs the actual state change
This enables multiplayer synchronization and replay recording.

**Context**: `IContext` (Context.h) is the main service locator providing access to audio, UI, objects, networking, scripting, etc.

**Scenes**: The game has distinct scenes (Preloader, Intro, Title, Game, Editor) managed through `IScene`.

## Code Style

- 4-space indentation, no tabs
- Allman brace style (braces on new lines)
- 128 character line limit
- `snake_case` for file names, `PascalCase` for types, `camelCase` for variables
- Use `#pragma once` for headers
- Warnings are errors (`-Werror` / `/WX`)

## Contributing

- Branch from and PR to `develop` (gitflow workflow)
- New localized strings go in `data/language/en-GB.txt` only
- See [Coding Style wiki](https://github.com/OpenRCT2/OpenRCT2/wiki/Coding-Style) for details
- Plugin API documented in `distribution/scripting.md` and `distribution/openrct2.d.ts`
