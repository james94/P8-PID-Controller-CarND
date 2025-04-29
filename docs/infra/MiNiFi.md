# PID Controller Edge Pipeline

Leveraged Perplexity AI to help create classes similar to MiNiFi processor approach: https://www.perplexity.ai/search/i-am-in-process-of-revamping-m-1vrgWljSSNujqrfT_k3BFw

## Network Communication Class (WebSocketHandler)

- Manages WebSocket connections and message routing
- Implements `hasData()` for message validation
- Handles connection/disconnection events
- Provides callback registration for message processing

## PID Controller Class (PIDProcessor)

- Generic PID implementation usable for both steering and throttle
- Separates error accumulation from output calculation
- Configurable gains through member functions

## Data Processor Class (TelemetryProcessor)

- Contains both PID controllers
- Manages vehicle state data
- Implements business logic for throttle/steering calculations
- Handles JSON data conversion

## Main Application Class (PID Control App)

- Composes WebSocket and Telemetry Processors
- Implements application lifecycle management
- Manages cross-component communication

## JSON Utilities (Namespace)

- Validation and error handling
- Default value handling
- Response formatting helpers

## Key Benefits of This Structure

**1. Separation of Concerns**

- Network I/O isolated in WebSocketHandler
- Control logic contained in PIDProcessor
- Business rules in TelemetryProcessor

**2. Testability**

- Mockable interfaces for unit testing
- Isolated PID algorithm testing
- JSON processing validation

**3. Extensibility**

- Easy to add new PID controllers
- Simple to implement new message types
- Straightforward to add logging/metrics

**4. Maintainability**

- Clear ownership of functionality
- Reduced code duplication
- Explicit data flow

