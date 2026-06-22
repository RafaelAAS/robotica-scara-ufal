{
  description = "Ambiente de Robótica (Cérebro Python para CoppeliaSim no Windows)";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-26.05";
  };

  outputs = { self, nixpkgs }:
  let
    system = "x86_64-linux";
    pkgs = import nixpkgs { inherit system; };

    # Criando o nosso Python "Cérebro" com as exatas vacinas que a robótica pede
    pythonComRobotica = pkgs.python3.withPackages (ps: with ps; [
      pyzmq     # A ponte de rede ZeroMQ para falar com o CoppeliaSim do Windows
      cbor2     # O tradutor de pacotes binários do Coppelia
      numpy     # O motor de álgebra linear e matrizes (essencial para robótica)
    ]);

  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [
        pythonComRobotica
        pkgs.git
      ];

      shellHook = ''
        echo "========================================================="
        echo " 🤖 AMBIENTE DE ROBÓTICA ATIVADO (WSL <-> Windows Mode)"
        echo "========================================================="
        echo " Bibliotecas injetadas no Python:"
        echo "   • pyzmq (ZeroMQ Remote API)"
        echo "   • cbor2"
        echo "   • numpy"
        echo "---------------------------------------------------------"
        echo " -> Para abrir a simulação: Abra o CoppeliaSim.exe normal no Windows."
        echo " -> Para programar: Digite 'python3 seu_script.py' aqui."
        echo "========================================================="
      '';
    };
  };
}