# Configurations
CRATE_NAME=layout
FRAMEWORK_NAME=LayoutEngineFFI
LIB_NAME=lib$(CRATE_NAME).a

.PHONY: all build-bindings build-xcframework clean

all: build-xcframework build-bindings

build-xcframework:
	# 1. Compile Rust targets for Apple architectures
	cargo build --manifest-path Layout/Cargo.toml --release --target aarch64-apple-ios
	cargo build --manifest-path Layout/Cargo.toml --release --target aarch64-apple-ios-sim
	cargo build --manifest-path Layout/Cargo.toml --release --target aarch64-apple-darwin

	# 2. Clear out old framework iterations
	rm -rf RustLayout/libs/$(FRAMEWORK_NAME).xcframework

	# 3. Create the unified Apple XCFramework
	xcodebuild -create-xcframework \
		-library Layout/target/aarch64-apple-ios/release/$(LIB_NAME) \
		-library Layout/target/aarch64-apple-ios-sim/release/$(LIB_NAME) \
		-library Layout/target/aarch64-apple-darwin/release/$(LIB_NAME) \
		-output RustLayout/libs/$(FRAMEWORK_NAME).xcframework

build-bindings:
	# 4. Generate UniFFI interface files and output directly to Swift source location
	rm -rf Sources/LayoutEngine/*
	cargo run --manifest-path Layout/Cargo.toml --bin uniffi-bindgen generate \
		Layout/src/uniffi.toml --language swift --out-dir Sources/LayoutEngine/

clean:
	cargo clean --manifest-path Layout/Cargo.toml
	rm -rf RustLayout/libs/*
	rm -rf Sources/LayoutEngine/*
