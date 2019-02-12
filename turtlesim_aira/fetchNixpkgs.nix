{ rev    ? "56139bee44a5d93d856081c8375509e2d096732c"             # The Git revision of nixpkgs to fetch
, sha256 ? "19r3wm03xnj38nfqjmd29k6dhqqh588paqqgnkbc73gaaq01lqlq" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
