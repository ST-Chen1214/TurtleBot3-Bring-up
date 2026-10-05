# Protocol Matrix

| Requirement | UDP | TCP | ZeroMQ | REST | Bluetooth |
|---|---:|---:|---:|---:|---:|
| Real-time velocity | **Primary** | no | no | no | no |
| Heartbeat | **Primary** | no | no | no | no |
| Reliable discrete command | no | **Primary** | no | no | local-only |
| Application ACK | no | **Primary** | no | HTTP response | text response |
| Continuous telemetry | no | no | **Primary** | no | no |
| Multiple telemetry subscribers | no | no | **Primary** | no | no |
| Camera snapshot | no | no | no | **Primary** | diagnostic health only |
| Full on-demand status | no | optional ACK only | continuous | **Primary** | local diagnostics |
| Commissioning without normal Wi-Fi path | no | no | no | no | **Primary** |

The matrix is intentionally opinionated so each communication mechanism has a defensible engineering purpose.
