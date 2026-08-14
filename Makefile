# Define variables for paths and targets
FEATURES = --features ffi
BINDGEN_FEATURES = --features bindgen-cli
LIB_NAME = liblayout.a

# Directory paths matching your layout
RUST_DIR = Layout
SWIFT_DIR = RustLayout
LIBS_DIR = $(SWIFT_DIR)/libs
BINDINGS_DIR = $(SWIFT_DIR)/bindings
HEADERS_DIR = $(BINDINGS_DIR)/headers
SWIFT_OUT_DIR = $(BINDINGS_DIR)/swift
XCFRAMEWORK_DIR = $(SWIFT_DIR)/Sources/Layout.xcframework
TARGET_DIR = $(RUST_DIR)/target

.PHONY: all macos ios xcframework clean

# Default lifecycle sequence
all: clean macos ios bindings xcframework

macos:
	@mkdir -p $(LIBS_DIR)
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release --lib $(FEATURES) --target aarch64-apple-darwin
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release --lib $(FEATURES) --target x86_64-apple-darwin
	@cargo +nightly build --manifest-path $(RUST_DIR)/Cargo.toml -Z build-std --release --lib $(FEATURES) --target aarch64-apple-ios-macabi
	@cargo +nightly build --manifest-path $(RUST_DIR)/Cargo.toml -Z build-std --release --lib $(FEATURES) --target x86_64-apple-ios-macabi
	@$(RM) -f $(LIBS_DIR)/liblayout-macos.a
	@$(RM) -f $(LIBS_DIR)/liblayout-maccatalyst.a
	@lipo -create -output $(LIBS_DIR)/liblayout-macos.a \
		$(TARGET_DIR)/aarch64-apple-darwin/release/$(LIB_NAME) \
		$(TARGET_DIR)/x86_64-apple-darwin/release/$(LIB_NAME)
	@lipo -create -output $(LIBS_DIR)/liblayout-maccatalyst.a \
		$(TARGET_DIR)/aarch64-apple-ios-macabi/release/$(LIB_NAME) \
		$(TARGET_DIR)/x86_64-apple-ios-macabi/release/$(LIB_NAME)

ios:
	@mkdir -p $(LIBS_DIR)
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release --lib $(FEATURES) --target aarch64-apple-ios
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release --lib $(FEATURES) --target aarch64-apple-ios-sim
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release --lib $(FEATURES) --target x86_64-apple-ios
	@$(RM) -f $(LIBS_DIR)/liblayout-ios.a
	@$(RM) -f $(LIBS_DIR)/liblayout-ios-sim.a
	@cp $(TARGET_DIR)/aarch64-apple-ios/release/$(LIB_NAME) $(LIBS_DIR)/liblayout-ios.a
	@lipo -create -output $(LIBS_DIR)/liblayout-ios-sim.a \
		$(TARGET_DIR)/aarch64-apple-ios-sim/release/$(LIB_NAME) \
		$(TARGET_DIR)/x86_64-apple-ios/release/$(LIB_NAME)

# Generates layout.swift + layoutFFI.h + layoutFFI.modulemap into one
# uniffi-bindgen output dir, then splits them: only the header + modulemap
# go where `xcframework` will pick them up as -headers (renamed to the
# canonical `module.modulemap` so Xcode's implicit module-map discovery
# finds it reliably).
bindings:
	@$(RM) -rf $(BINDINGS_DIR)
	@mkdir -p $(HEADERS_DIR) $(SWIFT_OUT_DIR)
	@cargo build --manifest-path $(RUST_DIR)/Cargo.toml --release $(FEATURES)
	@cd $(RUST_DIR) && cargo run $(BINDGEN_FEATURES) --bin uniffi-bindgen -- generate \
		--library target/release/$(LIB_NAME) \
		--language swift --out-dir ../$(BINDINGS_DIR)
	# UniFFI 0.28 generates `private var initializationResult = { ... }()` for
	# the bindings/scaffolding contract-version check. It's assigned exactly
	# once inside that immediately-invoked closure and never mutated again,
	# so Swift 6 (correctly) warns it should be `let`. Patch it here, before
	# the file is split/copied anywhere downstream, so every copy is already
	# fixed. Safe to drop once a future uniffi release fixes this upstream.
	@sed -i '' 's/private var initializationResult/private let initializationResult/g' \
		$(BINDINGS_DIR)/*.swift
	@mv $(BINDINGS_DIR)/*.h $(HEADERS_DIR)/
	@mv $(BINDINGS_DIR)/*.modulemap $(HEADERS_DIR)/module.modulemap
	@mv $(BINDINGS_DIR)/*.swift $(SWIFT_OUT_DIR)/

xcframework:
	@$(RM) -rf $(XCFRAMEWORK_DIR)
	@xcodebuild -create-xcframework \
		-library $(LIBS_DIR)/liblayout-macos.a -headers $(HEADERS_DIR) \
		-library $(LIBS_DIR)/liblayout-maccatalyst.a -headers $(HEADERS_DIR) \
		-library $(LIBS_DIR)/liblayout-ios.a -headers $(HEADERS_DIR) \
		-library $(LIBS_DIR)/liblayout-ios-sim.a -headers $(HEADERS_DIR) \
		-output $(XCFRAMEWORK_DIR)

clean:
	@cargo clean --manifest-path $(RUST_DIR)/Cargo.toml
	@$(RM) -rf $(LIBS_DIR) $(BINDINGS_DIR) $(XCFRAMEWORK_DIR)
