# libmav-mavsdk

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

## Usage in MAVSDK

```cpp
#include "mav/MessageSet.h"
#include "mav/Message.h"

// Load message definitions
mav::MessageSet message_set;
message_set.addFromXML("/path/to/common.xml");

// Create message safely (no exceptions)
auto message_opt = message_set.tryCreate("HEARTBEAT");
if (message_opt) {
    auto message = message_opt.value();
    
    // Set fields safely
    auto type_enum = message_set.tryGetEnum("MAV_TYPE_ONBOARD_CONTROLLER");
    if (type_enum) {
        message.trySet("type", static_cast<uint8_t>(type_enum.value()));
    }
    
    // Serialize for MAVSDK transport
    mav::Identifier sender{1, 1};
    auto length_opt = message.tryFinalize(seq_num, sender);
    if (length_opt) {
        // Send via MAVSDK: mavsdk_send(message.data(), length_opt.value());
    }
}
```

## Build

```bash
./build-with-deps.sh
./build/tests/test_no_exceptions  # Verify exception-free functionality
```

