idf_component_register(
    SRCS
        "main.cpp"
        "ftms_parser.cpp"
        "crank_synth.cpp"
        "cps_server.cpp"
        "ftms_client.cpp"
        "shared_state.cpp"
        "power_filter.cpp"
        "status_led.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES bt nvs_flash esp_timer esp_driver_gpio esp_app_format
)

# The project targets C++17 (IDF 5.3 defaults to gnu++2b; the last -std wins).
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
