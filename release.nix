{ nixpkgs ? import ./fetchNixpkgs.nix { }
, system ? builtins.currentSystem
}:

let
  pkgs = import nixpkgs { inherit system; };

in rec {
  fuji_weather = pkgs.callPackage ./fuji_weather { };
  fuji_weather_acl = pkgs.callPackage ./fuji_weather_acl { };
  iot_agent = pkgs.callPackage ./iot_agent { };
  turtlesim_aira = pkgs.callPackage ./turtlesim_aira { };
}
