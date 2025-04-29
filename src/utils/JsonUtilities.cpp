bool JsonUtils::validateTelemetry(const json& j) {
    const std::vector<std::string> required = {"speed", "steering_angle"};

    return std::all_of(required.begin(), required.end(),
        [&j](const auto& key) {
            return j.contains(key);
        });
}