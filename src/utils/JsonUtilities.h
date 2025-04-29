
/*
##
# JSON Utilities (Namespace)
##

- Validation and error handling
- Default value handling
- Response formatting helpers
*/

namespace JsonUtils {
    json createResponse(double steering, double throttle);
    bool validateTelemetry(const json& j);
    double parseWithDefault(const json& j, const std::string& key, double def);
}
