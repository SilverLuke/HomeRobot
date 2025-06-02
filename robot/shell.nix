
{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    python312
    python312Packages.matplotlib
    libusb1
    libudev-zero
    zlib
    patchelf
  ];
}

