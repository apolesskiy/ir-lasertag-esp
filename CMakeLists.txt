# IR Laser Tag Component
# Provides both low-level IR codec and protocol-specific APIs.
idf_component_register(
  SRCS
    # Codec layer (low-level encode/decode)
    "src/codec/ir_protocol_config.cpp"
    "src/codec/ir_encoder.cpp"
    "src/codec/ir_transmitter.cpp"
    "src/codec/ir_receiver.cpp"
    # Protocol layer (protocol-specific message APIs)
    "src/protocols/arclite/arclite.cpp"
    "src/protocols/irt/irt.cpp"
  INCLUDE_DIRS
    "include"
  REQUIRES
    log
    driver
    freertos
    esp_timer
)
