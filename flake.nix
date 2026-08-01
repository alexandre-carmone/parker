{
  description = "INDI solar/planetary imaging suite (Rust)";

  # Follow the host's pinned nixpkgs from the flake registry so the dev shell
  # resolves offline and matches the system.
  inputs.nixpkgs.url = "flake:nixpkgs";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };

      # Runtime libraries eframe/egui (winit + wgpu/glow) dlopen at runtime.
      runtimeLibs = with pkgs; [
        libglvnd            # libGL / libEGL
        wayland
        libxkbcommon
        libx11
        libxcursor
        libxrandr
        libxi
        vulkan-loader
        fontconfig
        freetype
      ];
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          rustc
          cargo
          rustfmt
          clippy
          rust-analyzer
          pkg-config
        ] ++ runtimeLibs;

        # winit/wgpu load these via dlopen, so they must be on the loader path.
        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath runtimeLibs;

        shellHook = ''
          echo "solar dev shell — rustc $(rustc --version 2>/dev/null | cut -d' ' -f2)"
          echo "Run the INDI simulators with:"
          echo "  indiserver -v indi_simulator_ccd indi_simulator_telescope indi_simulator_guide"
        '';
      };
    };
}
