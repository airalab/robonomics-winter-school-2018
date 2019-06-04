{ rev    ? "74eac314cc1cb815fb42f4316e6a29b0d9f9a8a3"             # The Git revision of nixpkgs to fetch
, sha256 ? "074ll7xhpl8h6pqsjl3src4822sccgnjz6nf9mg6k1h0lq06yrxc" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
