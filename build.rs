use std::env;
use std::path::PathBuf;

fn main() {
    let bindings = bindgen::Builder::default()
        .header("/Users/edwardsharp/src/misc/Pico-PIO-USB/src/pio_usb.h") // Adjust the path to the header file
        // .header("/Users/edwardsharp/src/misc/Adafruit_TinyUSB_Arduino/src/portable/raspberrypi/rp2040/rp2040_usb.h")
        // .header("/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/src/tusb.h")
        // .clang_arg(format!(
        //     "-I{}",
        //     "/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/examples/device/midi_test/src"
        // ))
        // // .header("/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/examples/device/midi_test/src/tusb_config.h")
        // .clang_arg(format!(
        //     "-I{}",
        //     "/Users/edwardsharp/src/misc/pico-sdk/src/rp2_common/pico_stdio_usb/include"
        // ))
        // .clang_arg(format!(
        //     "-I{}",
        //     "/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/src"
        // ))
        // .clang_arg(format!(
        //     "-I{}",
        //     "/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/arm-none-eabi/include"
        // ))
        .use_core()
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    // println!("ZOMG_OUT_DIR: {:?}", out_path);

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    // Compile the C library
    cc::Build::new()
        .compiler("/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc")
        .file("/Users/edwardsharp/src/misc/Pico-PIO-USB/src/pio_usb.h") // Adjust the path to the source file
        // .include("path/to/Pico-PIO-USB/include") // Adjust the path to the include directory
        .compile("pio_usb");

    // cc::Build::new()
    //     .compiler("/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc")
    //     .file("/Users/edwardsharp/src/misc/Adafruit_TinyUSB_Arduino/src/portable/raspberrypi/rp2040/rp2040_usb.h") // Adjust the path to the source file
    //     // .include("path/to/Pico-PIO-USB/include") // Adjust the path to the include directory
    //     .compile("rp2040_usb");
    // cc::Build::new()
    //     .compiler("/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/bin/arm-none-eabi-gcc")
    //     .file("/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/src/tusb.h") // Adjust the path to the source file
    //     .include("/Applications/ArmGNUToolchain/13.3.rel1/arm-none-eabi/arm-none-eabi/include") // Adjust the path to the include directory
    //     // .include("/Users/edwardsharp/src/misc/pico-sdk/lib/tinyusb/src")
    //     // .include("/Users/edwardsharp/src/misc/pico-sdk/src/host/pico_stdio")
    //     .compile("pio_usb");
}
