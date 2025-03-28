# FileMaker TLV Plugin for Ingenico Payment Terminals

## Project Structure

```
/filemaker-tlv-plugin/
├── CMakeLists.txt         - Build configuration
├── LICENSE                - MIT License
├── README.md              - This documentation
├── resources/
│   └── Example.fmp12.txt  - Sample FileMaker database
└── src/
    ├── filemaker/
    │   └── plugin.cpp     - FileMaker plugin interface
    ├── serial/            - Serial communication
    │   ├── crc16.cpp      - CRC validation
    │   ├── port_unix.cpp  - macOS/Linux impl
    │   ├── port_win.cpp   - Windows impl
    │   └── serial_port.h  - Interface
    └── tlv/               - TLV protocol
        ├── decoder.cpp    - TLV decoding
        ├── encoder.cpp    - TLV encoding
        └── tags.h         - EMV tag definitions
```

## How to Save Your Work
1. Create this folder structure on your local machine
2. Copy each file's content as provided
3. Save with matching filenames
4. The project will be ready to continue tomorrow