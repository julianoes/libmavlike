# libmavlike

**This is a hard fork of libmav specifically created for MAVSDK integration.**

This version has been stripped down to provide only the core MAVLink message manipulation API needed by MAVSDK. All networking, connection management, and exception-throwing components have been removed to ensure compatibility with MAVSDK's exception-free architecture.

**Key Changes from Upstream:**
- Switched from RapidXML to tinyxml2 for robust XML parsing with proper error handling
- Removed all networking interfaces (TCP, UDP, Serial)
- Removed Connection and NetworkRuntime classes
- Added comprehensive no-exceptions API (`tryCreate`, `trySet`, `tryGet`, `tryFinalize`)
- Exception-free internal code paths for MAVSDK compatibility

## Purpose

This fork provides MAVSDK with:
- Exception-free MAVLink message creation and manipulation
- Runtime XML message definition loading
- Type-safe field access and serialization
- No external dependencies on networking or threading libraries

## Build

```bash
./build-with-deps.sh
./build/tests/test
```

