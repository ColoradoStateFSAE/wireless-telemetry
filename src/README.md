# Generating `*.pb.h/c` files

Download [Nanopb](https://github.com/nanopb/nanopb/releases/) and use the packaged `protoc` to compile the schema file to C++ headers.
`protoc --nanopb_out=. telemetry.proto`